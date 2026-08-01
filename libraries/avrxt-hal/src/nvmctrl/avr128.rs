//! AVR128 DA/DB: USERROW (flash-tech) and flash self-programming.

use super::{Nvm, NvmError, NvmInstance, check_bounds};
use crate::clock::CcpUnlock;

/// AVR128-only flash capability on top of [`NvmInstance`]. tinyAVR USERROW is
/// EEPROM and it never self-programs flash.
///
/// # Safety
///
/// Same invariant as [`NvmInstance`]: the constants must match the actual
/// hardware.
pub unsafe trait FlashInstance: NvmInstance {
    /// Program flash page size in bytes (AVR Dx: 512).
    const FLASH_PAGE_SIZE: u32;
    /// Total program flash in bytes (AVR128: 128 KiB).
    const FLASH_SIZE: u32;

    /// Writes `CMD = FLPER` (flash page erase). SPM window first.
    fn command_flash_page_erase(&self);
    /// Writes `CMD = FLWR` (flash write). SPM window first.
    fn command_flash_write(&self);
    /// Writes `CTRLB.FLMAP` to select the visible 32 KiB flash section. IOREG
    /// window first. Sections 0-3 map to flash blocks starting at `0x0000`,
    /// `0x8000`, `0x10000`, `0x18000`.
    fn set_flmap(&self, section: u8);
}

/// Checks that `[offset, offset + len)` fits within the program flash.
fn check_flash_bounds<T: FlashInstance>(offset: u32, len: usize) -> Result<(), NvmError> {
    let len = u32::try_from(len).map_err(|_| NvmError::OutOfBounds)?;
    let end = offset.checked_add(len).ok_or(NvmError::OutOfBounds)?;
    if end > T::FLASH_SIZE {
        Err(NvmError::OutOfBounds)
    } else {
        Ok(())
    }
}

impl<T: FlashInstance> Nvm<T> {
    /// Runs `cmd` under IOREG unlock with interrupts masked. Used for
    /// `CTRLB.FLMAP`.
    fn protected_ioreg<C: CcpUnlock>(&self, cpu: &C, cmd: impl FnOnce(&T)) {
        avr_device::interrupt::free(|_| {
            cpu.unlock_ioreg();
            cmd(&self.instance);
        });
    }

    /// Data-space base of the mapped flash window.
    const FLASH_WINDOW_BASE: usize = 0x8000;
    /// Flash bytes mapped per `CTRLB.FLMAP` section (32 KiB).
    const FLASH_WINDOW_SIZE: u32 = 0x8000;

    /// Erases the USERROW and writes `data` from its start.
    ///
    /// The USERROW is flash technology and a single page. Erases the whole
    /// page, then writes. Bytes past `data` are left erased (`0xFF`).
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

    /// Erases the flash page starting at `flash_offset`.
    ///
    /// `flash_offset` must be page-aligned (`T::FLASH_PAGE_SIZE`) and within
    /// flash. Changes `CTRLB.FLMAP` to the section of `flash_offset`. The
    /// mapping is left set on return.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if `flash_offset` is not page-aligned or
    /// leaves flash, or [`NvmError::WriteFailed`] if the controller flags a
    /// write error.
    pub fn erase_flash_page<C: CcpUnlock>(
        &self,
        cpu: &C,
        flash_offset: u32,
    ) -> Result<(), NvmError> {
        if !flash_offset.is_multiple_of(T::FLASH_PAGE_SIZE) {
            return Err(NvmError::OutOfBounds);
        }
        let page_end = flash_offset
            .checked_add(T::FLASH_PAGE_SIZE)
            .ok_or(NvmError::OutOfBounds)?;
        if page_end > T::FLASH_SIZE {
            return Err(NvmError::OutOfBounds);
        }

        let section = u8::try_from(flash_offset / Self::FLASH_WINDOW_SIZE).unwrap_or(0);
        self.protected_ioreg(cpu, |inst| inst.set_flmap(section));

        self.instance.wait_flash_ready();
        self.protected(cpu, T::command_flash_page_erase);
        let addr = (Self::FLASH_WINDOW_BASE
            + usize::try_from(flash_offset % Self::FLASH_WINDOW_SIZE).unwrap_or(0))
            as *mut u8;
        // SAFETY: `addr` lands in the mapped flash window selected by `section`,
        // and the bounds check above kept the page inside flash.
        unsafe { addr.write_volatile(0xFF) };
        self.instance.wait_flash_ready();
        self.protected(cpu, T::command_none);
        if self.instance.write_error() {
            return Err(NvmError::WriteFailed);
        }
        Ok(())
    }

    /// Writes `data` to flash at `flash_offset`.
    ///
    /// The caller must erase every touched page first
    /// ([`Self::erase_flash_page`]): flash writes can only clear bits set by an
    /// erase. Changes `CTRLB.FLMAP` per section. The mapping is left set on
    /// return.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if the range leaves flash, or
    /// [`NvmError::WriteFailed`] if the controller flags a write error.
    pub fn write_flash<C: CcpUnlock>(
        &self,
        cpu: &C,
        flash_offset: u32,
        data: &[u8],
    ) -> Result<(), NvmError> {
        check_flash_bounds::<T>(flash_offset, data.len())?;

        let mut offset = flash_offset;
        let mut rest = data;
        while !rest.is_empty() {
            let section_end = (offset / Self::FLASH_WINDOW_SIZE + 1) * Self::FLASH_WINDOW_SIZE;
            let room = usize::try_from(section_end - offset).unwrap_or(usize::MAX);
            let n = rest.len().min(room);
            let (chunk, tail) = rest.split_at(n);
            self.write_flash_section(cpu, offset, chunk)?;
            offset = offset.saturating_add(u32::try_from(chunk.len()).unwrap_or(u32::MAX));
            rest = tail;
        }
        Ok(())
    }

    /// Writes `chunk` within a single FLMAP section. Splits at page boundaries
    /// and re-arms `FLWR` per page.
    fn write_flash_section<C: CcpUnlock>(
        &self,
        cpu: &C,
        flash_offset: u32,
        chunk: &[u8],
    ) -> Result<(), NvmError> {
        let section = u8::try_from(flash_offset / Self::FLASH_WINDOW_SIZE).unwrap_or(0);
        self.protected_ioreg(cpu, |inst| inst.set_flmap(section));

        self.instance.wait_flash_ready();

        let mut offset = flash_offset;
        let mut rest = chunk;
        while !rest.is_empty() {
            let page_end = (offset / T::FLASH_PAGE_SIZE + 1) * T::FLASH_PAGE_SIZE;
            let room = usize::try_from(page_end - offset).unwrap_or(usize::MAX);
            let n = rest.len().min(room);
            let (page_chunk, tail) = rest.split_at(n);

            let mut addr = (Self::FLASH_WINDOW_BASE
                + usize::try_from(offset % Self::FLASH_WINDOW_SIZE).unwrap_or(0))
                as *mut u8;

            self.protected(cpu, T::command_flash_write);
            for &b in page_chunk {
                // SAFETY: `addr` stays within the mapped flash window for
                // `section`, because `chunk` was split at the section boundary
                // and `page_chunk` was split at the page boundary.
                unsafe { addr.write_volatile(b) };
                addr = addr.wrapping_add(1);
            }
            self.instance.wait_flash_ready();
            self.protected(cpu, T::command_none);
            if self.instance.write_error() {
                return Err(NvmError::WriteFailed);
            }

            offset = offset.saturating_add(u32::try_from(n).unwrap_or(u32::MAX));
            rest = tail;
        }
        Ok(())
    }

    /// Reads `buf.len()` bytes from flash at `flash_offset`.
    ///
    /// Changes `CTRLB.FLMAP` per section. The mapping is left set on return.
    ///
    /// # Errors
    ///
    /// [`NvmError::OutOfBounds`] if the range leaves flash.
    pub fn read_flash<C: CcpUnlock>(
        &self,
        cpu: &C,
        flash_offset: u32,
        buf: &mut [u8],
    ) -> Result<(), NvmError> {
        check_flash_bounds::<T>(flash_offset, buf.len())?;

        let mut offset = flash_offset;
        let mut rest = buf;
        while !rest.is_empty() {
            let section = u8::try_from(offset / Self::FLASH_WINDOW_SIZE).unwrap_or(0);
            let section_end = (offset / Self::FLASH_WINDOW_SIZE + 1) * Self::FLASH_WINDOW_SIZE;
            let room = usize::try_from(section_end - offset).unwrap_or(usize::MAX);
            let n = rest.len().min(room);
            let (chunk, tail) = rest.split_at_mut(n);
            self.instance.wait_flash_ready();
            self.protected_ioreg(cpu, |inst| inst.set_flmap(section));
            let mut addr = (Self::FLASH_WINDOW_BASE
                + usize::try_from(offset % Self::FLASH_WINDOW_SIZE).unwrap_or(0))
                as *mut u8;
            for slot in chunk.iter_mut() {
                // SAFETY: `addr` stays within the mapped flash window for
                // `section`, because `chunk` was split at the section boundary.
                *slot = unsafe { addr.read_volatile() };
                addr = addr.wrapping_add(1);
            }
            offset = offset.saturating_add(u32::try_from(chunk.len()).unwrap_or(u32::MAX));
            rest = tail;
        }
        Ok(())
    }
}

macro_rules! impl_nvm_instance {
    ($NVMCTRL:ty) => {
        // SAFETY: addresses and sizes match the AVR128 DA/DB data-space map.
        unsafe impl NvmInstance for $NVMCTRL {
            const EEPROM_START: *mut u8 = 0x1400 as *mut u8;
            const EEPROM_SIZE: u16 = 512;
            const USERROW_START: *mut u8 = 0x1080 as *mut u8;
            const USERROW_SIZE: u16 = 32;
            const EEPROM_ARM_FIRST: bool = true;

            #[inline(always)]
            fn wait_flash_ready(&self) {
                crate::wait::spin_until(|| self.status().read().fbusy().bit_is_clear());
            }
            #[inline(always)]
            fn wait_eeprom_ready(&self) {
                crate::wait::spin_until(|| self.status().read().eebusy().bit_is_clear());
            }
            #[inline(always)]
            fn write_error(&self) -> bool {
                !self.status().read().error().is_noerror()
            }
            #[inline(always)]
            fn command_eeprom_erase_write(&self) {
                self.ctrla().write(|w| w.cmd().eeerwr());
            }
            #[inline(always)]
            fn command_none(&self) {
                self.ctrla().write(|w| w.cmd().none());
            }
        }

        // SAFETY: flash parameters match AVR128 DA/DB (512 B pages, 128 KiB).
        unsafe impl FlashInstance for $NVMCTRL {
            const FLASH_PAGE_SIZE: u32 = 512;
            const FLASH_SIZE: u32 = 128 * 1024;

            #[inline(always)]
            fn command_flash_page_erase(&self) {
                self.ctrla().write(|w| w.cmd().flper());
            }
            #[inline(always)]
            fn command_flash_write(&self) {
                self.ctrla().write(|w| w.cmd().flwr());
            }
            #[inline(always)]
            fn set_flmap(&self, section: u8) {
                self.ctrlb().write(|w| match section & 0x3 {
                    0 => w.flmap().section0(),
                    1 => w.flmap().section1(),
                    2 => w.flmap().section2(),
                    _ => w.flmap().section3(),
                });
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

const _: () = {
    assert!(check_bounds(0, 512, 512).is_ok());
    assert!(check_bounds(508, 4, 512).is_ok());
    assert!(check_bounds(512, 0, 512).is_ok());
    assert!(check_bounds(0, 513, 512).is_err());
    assert!(check_bounds(509, 4, 512).is_err());
    assert!(check_bounds(u16::MAX, 2, 512).is_err());
};
