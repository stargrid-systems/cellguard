//! Persistent panic record and crash-loop policy for the `CellGuard` firmware.
//!
//! A panic handler stores a [`PanicRecord`] (panic location, reset-cause
//! flags, crash-loop counter) in on-chip EEPROM, then resets the device. After
//! a configurable number of consecutive panic-resets the policy halts instead.
//! The record format is pure and host-testable. The NVM-backed storage and
//! the reset/halt decision live behind the `hal` feature.
//!
//! # Features
//!
//! - `hal`: EEPROM-backed panic storage and the `store_and_decide` policy.
//!   Requires `avrxt-hal`.

#![no_std]

pub use self::record::{FILE_CAP, PanicRecord, RECORD_FORMAT_VERSION, RECORD_LEN};
#[cfg(feature = "hal")]
pub use self::store::{Decision, clear, read_panic_record, store_and_decide};

mod record;
#[cfg(feature = "hal")]
mod store;

/// Defines a standard `#[panic_handler]` for a `CellGuard` firmware crate.
///
/// The handler disables interrupts, records the panic via
/// [`store_and_decide`], then resets or halts per the decision. Reset-cause
/// flags come from `RSTCTRL.RSTFR`. `$steal_peripherals` takes the device
/// peripherals, for example
/// `unsafe { avr_device::avr128da64::Peripherals::steal() }`. `$offset` is
/// the EEPROM panic-record slot and `$threshold` the crash-loop limit.
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
