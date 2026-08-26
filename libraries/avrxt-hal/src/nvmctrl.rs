//! Non-volatile memory controller (NVMCTRL) for AVR128 DA/DB and tinyAVR
//! 0/1-series.
//!
//! [`Nvm`] reads and writes the on-chip EEPROM. On AVR128 it also writes the
//! USERROW and self-programs flash. On tinyAVR 0/1-series it additionally
//! exposes the flash self-write primitives (`load_flash_byte`,
//! `erase_write_flash_page`) for the self-update apply path, which is not
//! implemented yet (issue #60).
//!
//! EEPROM write models differ by family. AVR128 arms `EEERWR` first, then each
//! store triggers a byte-level erase-write. tinyAVR stores first, then `ERWP`
//! commits. Both map onto [`NvmInstance::command_eeprom_erase_write`].
//!
//! AVR128 USERROW is flash technology (erase-then-write). tinyAVR USERROW is
//! plain EEPROM. AVR128 flash self-programming remaps a 32 KiB data-space
//! window over 128 KiB in four `CTRLB.FLMAP` sections.
//!
//! `NVMCTRL.CTRLA` is SPM-protected. Every command goes through
//! [`CcpUnlock::unlock_spm`] inside `avr_device::interrupt::free`. Do not run
//! NVM operations from an interrupt handler: a store to a mapped region loads
//! the shared page buffer, which an ISR could corrupt mid-write.
//!
//! tinyAVR note (`ATtiny416` errata DS80000933 2.6.1): `CTRLA` may read
//! non-zero after reset. The code always writes the command explicitly.

use core::mem::MaybeUninit;

#[cfg(feature = "_avr128")]
pub use self::avr128::FlashInstance;
#[cfg(feature = "_tinyavr")]
pub use self::tinyavr::{FLASH_BASE, FLASH_PAGE_SIZE, FLASH_SIZE};
use crate::clock::CcpUnlock;

#[cfg(feature = "_avr128")]
mod avr128;
#[cfg(feature = "_tinyavr")]
mod tinyavr;

/// Something went wrong talking to non-volatile memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum NvmError {
    /// The requested range does not fit the target region.
    OutOfBounds,
    /// The controller reported a write error (`STATUS.WRERROR` / `.ERROR`).
    WriteFailed,
}

/// An `NVMCTRL` peripheral. Implemented for each AVR128 and tinyAVR device. Not
/// for external use.
///
/// # Safety
///
/// The pointer constants must be valid data-space addresses of the respective
/// mapped regions. Wrong values cause out-of-bounds memory access in safe code
/// that calls [`Nvm`] read/write methods.
pub unsafe trait NvmInstance {
    /// Data-space base pointer of the on-chip EEPROM.
    const EEPROM_START: *mut u8;
    /// On-chip EEPROM size in bytes.
    const EEPROM_SIZE: u16;
    /// Data-space base pointer of the USERROW.
    const USERROW_START: *mut u8;
    /// USERROW size in bytes.
    const USERROW_SIZE: u16;
    /// Whether the EEPROM erase-write command must be armed before the store
    /// (AVR128: `true`) or after each store (tinyAVR: `false`).
    const EEPROM_ARM_FIRST: bool;

    /// Spins until the flash/USERROW controller is idle.
    fn wait_flash_ready(&self);
    /// Spins until the EEPROM controller is idle.
    fn wait_eeprom_ready(&self);
    /// Returns `true` if `STATUS` flags a write error.
    fn write_error(&self) -> bool;

    /// Writes the erase-write command for an EEPROM byte/page. AVR128:
    /// `EEERWR`, tinyAVR: `ERWP`. Caller must open the SPM window first.
    fn command_eeprom_erase_write(&self);
    /// Writes `CMD = NONE` (disarm). Caller must open the SPM window first.
    fn command_none(&self);
}

/// The non-volatile memory controller.
pub struct Nvm<T: NvmInstance> {
    instance: T,
}

impl<T: NvmInstance> Nvm<T> {
    /// Takes ownership of the `NVMCTRL` peripheral.
    #[must_use]
    pub const fn new(instance: T) -> Self {
        Self { instance }
    }

    /// Returns the wrapped peripheral.
    #[must_use]
    pub fn release(self) -> T {
        self.instance
    }

    /// Runs `cmd` under SPM unlock with interrupts masked.
    fn protected<C: CcpUnlock>(&self, cpu: &C, cmd: impl FnOnce(&T)) {
        avr_device::interrupt::free(|_| {
            cpu.unlock_spm();
            cmd(&self.instance);
        });
    }

    /// Reads `buf.len()` bytes from the on-chip EEPROM at `offset`.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if the range leaves the EEPROM.
    pub fn read_eeprom(&self, offset: u16, buf: &mut [u8]) -> Result<(), NvmError> {
        self.read_eeprom_uninit(offset, as_uninit(buf))?;
        Ok(())
    }

    /// Reads `buf.len()` bytes from the on-chip EEPROM at `offset` into an
    /// uninitialized buffer, returning the bytes that were read.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if the range leaves the EEPROM.
    pub fn read_eeprom_uninit<'a>(
        &self,
        offset: u16,
        buf: &'a mut [MaybeUninit<u8>],
    ) -> Result<&'a mut [u8], NvmError> {
        // SAFETY: `EEPROM_START` bases the mapped EEPROM of `EEPROM_SIZE` bytes.
        unsafe { read_region(offset, buf, T::EEPROM_START, T::EEPROM_SIZE) }
    }

    /// Writes `data` to the on-chip EEPROM at `offset`.
    ///
    /// Per-byte erase-write, so callers do not pre-erase.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if the range leaves the EEPROM, or
    /// [`NvmError::WriteFailed`] if the controller flags a write error.
    pub fn write_eeprom<C: CcpUnlock>(
        &self,
        offset: u16,
        data: &[u8],
        cpu: &C,
    ) -> Result<(), NvmError> {
        check_bounds(offset, data.len(), T::EEPROM_SIZE)?;
        let mut addr = T::EEPROM_START.wrapping_add(offset as usize);

        for &b in data {
            self.instance.wait_eeprom_ready();
            // AVR128DA/DB silicon only commits one erase-write per EEERWR
            // arm despite the datasheet allowing several, and re-arming
            // while still armed raises WRERROR. So arm and disarm around
            // every byte. tinyAVR arms after the store instead.
            if T::EEPROM_ARM_FIRST {
                self.protected(cpu, T::command_eeprom_erase_write);
            }
            // SAFETY: `check_bounds` kept `offset + data.len()` inside the
            // EEPROM, so every `addr` stays within the mapped region.
            unsafe { addr.write_volatile(b) };
            if !T::EEPROM_ARM_FIRST {
                self.protected(cpu, T::command_eeprom_erase_write);
            }
            self.instance.wait_eeprom_ready();
            if self.instance.write_error() {
                self.protected(cpu, T::command_none);
                return Err(NvmError::WriteFailed);
            }
            if T::EEPROM_ARM_FIRST {
                self.protected(cpu, T::command_none);
            }
            addr = addr.wrapping_add(1);
        }

        self.protected(cpu, T::command_none);
        Ok(())
    }

    /// Reads `buf.len()` bytes from the USERROW at `offset`.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if the range leaves the USERROW.
    pub fn read_userrow(&self, offset: u16, buf: &mut [u8]) -> Result<(), NvmError> {
        self.read_userrow_uninit(offset, as_uninit(buf))?;
        Ok(())
    }

    /// Reads `buf.len()` bytes from the USERROW at `offset` into an
    /// uninitialized buffer, returning the bytes that were read.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if the range leaves the USERROW.
    pub fn read_userrow_uninit<'a>(
        &self,
        offset: u16,
        buf: &'a mut [MaybeUninit<u8>],
    ) -> Result<&'a mut [u8], NvmError> {
        // SAFETY: `USERROW_START` bases the mapped USERROW of `USERROW_SIZE`
        // bytes.
        unsafe { read_region(offset, buf, T::USERROW_START, T::USERROW_SIZE) }
    }
}

/// Checks that `[offset, offset + len)` fits within a region of `size` bytes.
const fn check_bounds(offset: u16, len: usize, size: u16) -> Result<(), NvmError> {
    match (offset as usize).checked_add(len) {
        Some(end) if end <= size as usize => Ok(()),
        _ => Err(NvmError::OutOfBounds),
    }
}

/// Views an initialized byte slice as uninitialized.
const fn as_uninit(buf: &mut [u8]) -> &mut [MaybeUninit<u8>] {
    let len = buf.len();
    // SAFETY: `MaybeUninit<u8>` has the same layout as `u8`.
    unsafe { core::slice::from_raw_parts_mut(buf.as_mut_ptr().cast(), len) }
}

/// Reads from the region at `start` into `buf`, returning the initialized
/// bytes.
///
/// # Safety
///
/// `start` must be the base of a readable data-space region of at least `size`
/// bytes, so that `[start, start + size)` is a valid mapped range.
///
/// # Errors
///
/// [`NvmError::OutOfBounds`] if the range leaves the region.
unsafe fn read_region(
    offset: u16,
    buf: &mut [MaybeUninit<u8>],
    start: *mut u8,
    size: u16,
) -> Result<&mut [u8], NvmError> {
    check_bounds(offset, buf.len(), size)?;
    let mut addr = start.wrapping_add(offset as usize);
    for slot in buf.iter_mut() {
        // SAFETY: the caller guarantees `start`/`size` map a real region, and
        // `check_bounds` kept every `addr` inside it.
        slot.write(unsafe { addr.read_volatile() });
        addr = addr.wrapping_add(1);
    }
    let len = buf.len();
    let ptr = buf.as_mut_ptr().cast::<u8>();
    // SAFETY: every slot in `buf` was written above, so the region is a fully
    // initialized `[u8]` for the borrow of `buf`.
    Ok(unsafe { core::slice::from_raw_parts_mut(ptr, len) })
}
