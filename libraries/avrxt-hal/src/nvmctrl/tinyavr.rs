//! tinyAVR 0/1-series: EEPROM only.

use super::NvmInstance;

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
};
