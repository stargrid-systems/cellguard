//! Persistent panic record and crash-loop policy for the `CellGuard` firmware.
//!
//! A panic handler stores a [`record::PanicRecord`] (the panic location plus
//! the reset-cause flags and a crash-loop counter) in on-chip EEPROM, then
//! resets the device. A later boot or a field-bus probe reads the record back
//! to learn where and why the device panicked. After a configurable number of
//! consecutive panic-resets the policy halts instead of resetting, so a
//! persistent fault cannot reboot-loop forever.
//!
//! The [`record`] module is pure and host-testable. The NVM-backed storage and
//! reset/halt decision live behind the `hal` feature.
//!
//! # Features
//!
//! - `hal` (off by default): EEPROM-backed panic storage and the
//!   [`store::store_and_decide`] policy. Requires `avrxt-hal`.

#![no_std]

pub use self::record::{FILE_CAP, PanicRecord, RECORD_FORMAT_VERSION, RECORD_LEN};

pub mod record;
#[cfg(feature = "hal")]
pub mod store;
#[cfg(feature = "hal")]
pub use self::store::{Decision, clear, store_and_decide};

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
