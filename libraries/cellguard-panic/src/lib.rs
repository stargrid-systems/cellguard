//! Persistent panic record and crash-loop policy for the `CellGuard` firmware.
//!
//! A panic handler stores a [`PanicRecord`] (the panic location plus
//! the reset-cause flags and a crash-loop counter) in on-chip EEPROM, then
//! resets the device. A later boot or a field-bus probe reads the record back
//! to learn where and why the device panicked. After a configurable number of
//! consecutive panic-resets the policy halts instead of resetting, so a
//! persistent fault cannot reboot-loop forever.
//!
//! The record module is pure and host-testable. The NVM-backed storage and
//! reset/halt decision live behind the `hal` feature.
//!
//! # Features
//!
//! - `hal` (off by default): EEPROM-backed panic storage and the
//!   [`store_and_decide`] policy. Requires `avrxt-hal`.

#![no_std]

pub use self::record::{FILE_CAP, PanicRecord, RECORD_FORMAT_VERSION, RECORD_LEN};
#[cfg(feature = "hal")]
pub use self::store::{Decision, clear, read_panic_record, store_and_decide};

mod record;
#[cfg(feature = "hal")]
mod store;

/// Defines a standard `#[panic_handler]` for a CellGuard firmware crate.
///
/// Expands to a handler that disables interrupts, steals the peripherals,
/// records the panic via [`store_and_decide`], then either resets (under the
/// crash-loop threshold) or halts. `$steal_peripherals` is the expression that
/// takes the device peripherals (e.g.
/// `unsafe { avr_device::avr128da64::Peripherals::steal() }`). `$offset` and
/// `$threshold` configure the EEPROM panic-record slot.
///
/// The handler also pulls the reset-cause flags from `RSTCTRL.RSTFR`, so the
/// record shows why the panic fired.
///
/// Requires the `hal` feature.
#[cfg(feature = "hal")]
#[macro_export]
macro_rules! panic_handler {
    ($steal_peripherals:expr, $offset:expr, $threshold:expr) => {
        #[panic_handler]
        fn panic(info: &core::panic::PanicInfo) -> ! {
            avr_device::interrupt::disable();
            let dp = $steal_peripherals;
            let nvm = avrxt_hal::nvmctrl::Nvm::new(dp.NVMCTRL);
            let flags = avrxt_hal::rstctrl::RstInstance::flags(&dp.RSTCTRL).bits();
            match $crate::store_and_decide(&nvm, &dp.CPU, $offset, $threshold, flags, info) {
                $crate::Decision::Reset => {
                    avrxt_hal::rstctrl::RstInstance::software_reset(&dp.RSTCTRL, &dp.CPU)
                }
                $crate::Decision::Halt => loop {
                    core::hint::spin_loop();
                },
            }
        }
    };
}

/// An error returned when a [`PanicRecord`] cannot be parsed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ParseError {
    /// The stored CRC did not match the contents.
    BadCrc,
    /// The record format version is not [`RECORD_FORMAT_VERSION`]. A blank
    /// `0xFF` EEPROM slot lands here.
    UnsupportedVersion(u8),
    /// A field held an out-of-range value.
    BadField,
}

impl core::fmt::Display for ParseError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BadCrc => f.write_str("panic record CRC mismatch"),
            Self::UnsupportedVersion(v) => {
                write!(f, "unsupported panic record format version {v}")
            }
            Self::BadField => f.write_str("panic record field out of range"),
        }
    }
}

impl core::error::Error for ParseError {}
