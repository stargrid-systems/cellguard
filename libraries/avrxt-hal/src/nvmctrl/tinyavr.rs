//! tinyAVR 0/1-series: EEPROM and flash self-write.

use super::NvmInstance;
use crate::clock::CcpUnlock;
use crate::nvmctrl::Nvm;

/// Base of program flash in the mapped data space.
pub const FLASH_BASE: u16 = 0x8000;
/// Total flash size for the 4 KiB tinyAVR 0/1-series parts.
pub const FLASH_SIZE: u16 = 4096;
/// Flash page size in bytes (erase and write unit).
pub const FLASH_PAGE_SIZE: u16 = 16;

macro_rules! impl_tiny_nvm_instance {
    ($NVMCTRL:ty) => {
        // SAFETY: addresses and sizes match the tinyAVR 0/1-series data-space map.
        unsafe impl NvmInstance for $NVMCTRL {
            const EEPROM_START: *mut u8 = 0x1400 as *mut u8;
            const EEPROM_SIZE: u16 = 128;
            const USERROW_START: *mut u8 = 0x1300 as *mut u8;
            const USERROW_SIZE: u16 = 32;
            const EEPROM_ARM_FIRST: bool = false;

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
                self.status().read().wrerror().bit_is_set()
            }
            #[inline(always)]
            fn command_eeprom_erase_write(&self) {
                self.ctrla().write(|w| w.cmd().erwp());
            }
            #[inline(always)]
            fn command_none(&self) {
                self.ctrla().write(|w| w.cmd().none());
            }
        }

        impl Nvm<$NVMCTRL> {
            /// Spins until the NVM controller is idle. The CPU is halted
            /// during a flash command anyway, so this only guards against a
            /// command left over from a faulted sequence.
            #[inline(always)]
            pub fn wait_nvm_idle(&self) {
                self.instance.wait_flash_ready();
            }

            /// Stores `byte` into the flash page buffer at flash `offset`.
            ///
            /// The mapped store both loads one page-buffer byte and latches
            /// the target page address, so the last byte stored decides which
            /// page [`Self::erase_write_flash_page`] commits. `offset` is a
            /// flash byte offset (`0..FLASH_SIZE`).
            #[inline(always)]
            pub fn load_flash_byte(&self, offset: u16, byte: u8) {
                let addr = FLASH_BASE.wrapping_add(offset) as *mut u8;
                // SAFETY: the mapped flash window is a valid data-space
                // region, and a store while no command is pending only loads
                // the page buffer.
                unsafe { addr.write_volatile(byte) };
            }

            /// Erase-and-writes the latched flash page in one command
            /// (`CTRLA.CMD = ERWP`), under SPM unlock.
            ///
            /// Interrupts must be disabled across the load and this call: an
            /// ISR between the CCP store and the command store closes the
            /// unlock window and the command is silently dropped.
            #[inline(always)]
            pub fn erase_write_flash_page<C: CcpUnlock>(&self, cpu: &C) {
                cpu.unlock_spm();
                self.instance.ctrla().write(|w| w.cmd().erwp());
            }
        }
    };
}

#[cfg(feature = "attiny406")]
impl_tiny_nvm_instance!(avr_device::attiny406::NVMCTRL);
#[cfg(feature = "attiny416")]
impl_tiny_nvm_instance!(avr_device::attiny416::NVMCTRL);

const _: () = {
    assert!(super::check_bounds(0, 128, 128).is_ok());
    assert!(super::check_bounds(124, 4, 128).is_ok());
    assert!(super::check_bounds(128, 0, 128).is_ok());
    assert!(super::check_bounds(0, 129, 128).is_err());
    assert!(super::check_bounds(u16::MAX, 2, 128).is_err());
    assert!(FLASH_BASE + FLASH_SIZE == 0x9000);
    assert!(FLASH_SIZE.is_multiple_of(FLASH_PAGE_SIZE));
};
