//! Adapters that back the [`io`](crate::io) traits with concrete device
//! drivers.
//!
//! These live here rather than in a driver crate because the orphan rule needs
//! the adapter to sit in a crate that owns either the trait or the type.
//! `cellboot` owns the I/O traits, so each adapter is a local newtype over a
//! foreign driver.
//!
//! - [`Cat25Store`] (feature `drivers`): [`ImageStore`](crate::io::ImageStore)
//!   over a CAT25 SPI EEPROM, shared by both the main MCU (which writes the
//!   staged image) and the PROG MCU (which reads it back).
//! - [`EepromState`], [`UserRowKeyStore`] (feature `avr128`): the AVR128
//!   on-chip [`StateStore`](crate::io::StateStore) and
//!   [`KeyStore`](crate::io::KeyStore).

#[cfg(feature = "avr128")]
pub use self::avr128::{EepromState, UserRowKeyStore};
#[cfg(feature = "drivers")]
pub use self::cat25::Cat25Store;

#[cfg(feature = "avr128")]
mod avr128;
#[cfg(feature = "drivers")]
mod cat25;
