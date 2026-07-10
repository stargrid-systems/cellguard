//! Non-volatile memory controller (NVMCTRL) for AVR128 DA/DB.
//!
//! [`Nvm`] programs the on-chip EEPROM and the USERROW. It is generic over an
//! [`NvmInstance`] (implemented for each AVR128 `NVMCTRL`). Program flash is
//! not written here: on `CellGuard` a separate programmer drives the target
//! over UPDI, so a running application never rewrites its own code.
//!
//! `NVMCTRL.CTRLA` is configuration-change protected with the SPM signature, so
//! every command goes through [`CcpUnlock::unlock_spm`] inside
//! `avr_device::interrupt::free`: an interrupt cannot land in the unlock
//! window. Do not run an NVM operation from an interrupt handler.
//!
//! The register command sequences here follow the AVR Dx model and match the
//! DxCore reference flows. They cannot be exercised without hardware, so treat
//! the marked sequences as bench-verify.

use crate::clock::CcpUnlock;

/// Something went wrong talking to non-volatile memory.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum NvmError {
    /// The requested range does not fit the target region.
    OutOfBounds,
    /// The controller reported a write error (`STATUS.ERROR`).
    WriteFailed,
}

/// Checks that `[offset, offset + len)` fits within a region of `size` bytes.
const fn check_bounds(offset: u16, len: usize, size: u16) -> Result<(), NvmError> {
    match (offset as usize).checked_add(len) {
        Some(end) if end <= size as usize => Ok(()),
        _ => Err(NvmError::OutOfBounds),
    }
}

/// An `NVMCTRL` peripheral. Implemented for each AVR128 device. Not for
/// external use.
///
/// The associated constants give each region's data-space base and size, so a
/// future part with a different map only changes these values.
pub trait NvmInstance {
    /// Data-space base pointer of the on-chip EEPROM.
    const EEPROM_START: *mut u8;
    /// On-chip EEPROM size in bytes.
    const EEPROM_SIZE: u16;
    /// Data-space base pointer of the USERROW.
    const USERROW_START: *mut u8;
    /// USERROW size in bytes.
    const USERROW_SIZE: u16;

    /// Spins until the flash/USERROW controller is idle (`!STATUS.FBUSY`).
    fn wait_flash_ready(&self);
    /// Spins until the EEPROM controller is idle (`!STATUS.EEBUSY`).
    fn wait_eeprom_ready(&self);
    /// Returns `true` if `STATUS.ERROR` flags a write error.
    fn write_error(&self) -> bool;

    /// Writes `CMD = NONE`. Caller must open the SPM window first.
    fn command_none(&self);
    /// Writes `CMD = EEERWR` (EEPROM erase-write). SPM window first.
    fn command_eeprom_erase_write(&self);
    /// Writes `CMD = FLPER` (flash page erase). SPM window first.
    fn command_flash_page_erase(&self);
    /// Writes `CMD = FLWR` (flash write). SPM window first.
    fn command_flash_write(&self);
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

    /// Runs `cmd` inside the SPM configuration-change window with interrupts
    /// masked, so the unlock and the protected store stay adjacent.
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
        check_bounds(offset, buf.len(), T::EEPROM_SIZE)?;
        let mut addr = T::EEPROM_START.wrapping_add(offset as usize);
        for slot in buf {
            // SAFETY: `check_bounds` kept `offset + buf.len()` inside the EEPROM,
            // so every `addr` stays within the mapped region.
            *slot = unsafe { addr.read_volatile() };
            addr = addr.wrapping_add(1);
        }
        Ok(())
    }

    /// Writes `data` to the on-chip EEPROM at `offset`.
    ///
    /// Uses the erase-write command per byte, so callers do not pre-erase. This
    /// follows the DxCore EEPROM flow: store the byte to load the page buffer,
    /// then issue `EEERWR` to commit it. Bench-verify.
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
            // SAFETY: `check_bounds` kept `offset + data.len()` inside the
            // EEPROM, so every `addr` stays within the mapped region. The store
            // loads the page buffer; the command below commits it.
            unsafe { addr.write_volatile(b) };
            self.protected(cpu, T::command_eeprom_erase_write);
            addr = addr.wrapping_add(1);
        }
        self.instance.wait_eeprom_ready();
        self.protected(cpu, T::command_none);
        if self.instance.write_error() {
            return Err(NvmError::WriteFailed);
        }
        Ok(())
    }

    /// Reads `buf.len()` bytes from the USERROW at `offset`.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if the range leaves the USERROW.
    pub fn read_userrow(&self, offset: u16, buf: &mut [u8]) -> Result<(), NvmError> {
        check_bounds(offset, buf.len(), T::USERROW_SIZE)?;
        let mut addr = T::USERROW_START.wrapping_add(offset as usize);
        for slot in buf {
            // SAFETY: `check_bounds` kept `offset + buf.len()` inside the
            // USERROW, so every `addr` stays within the mapped region.
            *slot = unsafe { addr.read_volatile() };
            addr = addr.wrapping_add(1);
        }
        Ok(())
    }

    /// Erases the USERROW and writes `data` from its start.
    ///
    /// The USERROW is flash technology and a single page, so this erases the
    /// whole page then writes `data`. Bytes past `data` are left erased
    /// (`0xFF`). Follows the AVR Dx flash flow (command-first): arm `FLPER`,
    /// trigger the erase with a store, then arm `FLWR` and store the bytes.
    /// Bench-verify.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if `data` is larger than the USERROW, or
    /// [`NvmError::WriteFailed`] if the controller flags a write error.
    pub fn write_userrow<C: CcpUnlock>(&self, data: &[u8], cpu: &C) -> Result<(), NvmError> {
        check_bounds(0, data.len(), T::USERROW_SIZE)?;

        self.instance.wait_flash_ready();
        self.protected(cpu, T::command_flash_page_erase);
        // SAFETY: `USERROW_START` is the base of the mapped USERROW region.
        unsafe { T::USERROW_START.write_volatile(0xFF) };
        self.instance.wait_flash_ready();

        self.protected(cpu, T::command_flash_write);
        let mut addr = T::USERROW_START;
        for &b in data {
            // SAFETY: `check_bounds` kept `data.len()` inside the USERROW, so
            // every `addr` stays within the mapped region.
            unsafe { addr.write_volatile(b) };
            addr = addr.wrapping_add(1);
        }
        self.instance.wait_flash_ready();

        self.protected(cpu, T::command_none);
        if self.instance.write_error() {
            return Err(NvmError::WriteFailed);
        }
        Ok(())
    }
}

macro_rules! impl_nvm_instance {
    ($NVMCTRL:ty) => {
        impl NvmInstance for $NVMCTRL {
            // AVR128 DA/DB data-space map (data sheet memory overview).
            const EEPROM_START: *mut u8 = 0x1400 as *mut u8;
            const EEPROM_SIZE: u16 = 512;
            const USERROW_START: *mut u8 = 0x1080 as *mut u8;
            const USERROW_SIZE: u16 = 32;

            #[inline(always)]
            fn wait_flash_ready(&self) {
                while self.status().read().fbusy().bit_is_set() {}
            }
            #[inline(always)]
            fn wait_eeprom_ready(&self) {
                while self.status().read().eebusy().bit_is_set() {}
            }
            #[inline(always)]
            fn write_error(&self) -> bool {
                !self.status().read().error().is_noerror()
            }
            #[inline(always)]
            fn command_none(&self) {
                self.ctrla().write(|w| w.cmd().none());
            }
            #[inline(always)]
            fn command_eeprom_erase_write(&self) {
                self.ctrla().write(|w| w.cmd().eeerwr());
            }
            #[inline(always)]
            fn command_flash_page_erase(&self) {
                self.ctrla().write(|w| w.cmd().flper());
            }
            #[inline(always)]
            fn command_flash_write(&self) {
                self.ctrla().write(|w| w.cmd().flwr());
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_nvm_instance!(avr_device::avr128db48::NVMCTRL);
#[cfg(feature = "avr128db64")]
impl_nvm_instance!(avr_device::avr128db64::NVMCTRL);
#[cfg(feature = "avr128da64")]
impl_nvm_instance!(avr_device::avr128da64::NVMCTRL);

// The HAL builds only for `avr-none`, so these guard `check_bounds` at compile
// time instead of a host test runner.
const _: () = {
    assert!(check_bounds(0, 512, 512).is_ok());
    assert!(check_bounds(508, 4, 512).is_ok());
    assert!(check_bounds(512, 0, 512).is_ok());
    assert!(check_bounds(0, 513, 512).is_err());
    assert!(check_bounds(509, 4, 512).is_err());
    assert!(check_bounds(u16::MAX, 2, 512).is_err());
};
