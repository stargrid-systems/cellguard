//! Adapters that back the [`io`](crate::io) traits with concrete drivers.
//!
//! They live here because the orphan rule puts the adapter in a crate that
//! owns either the trait or the type: each is a newtype over a foreign driver.
//!
//! - [`Cat25Store`] (feature `drivers`): [`ImageStore`](crate::io::ImageStore)
//!   over a CAT25 SPI EEPROM.
//! - [`EepromState`], [`UserRowKeyStore`], [`FlashNvmWriter`] (feature
//!   `avr128`): the AVR128 on-chip [`StateStore`](crate::io::StateStore),
//!   [`KeyStore`](crate::io::KeyStore), and
//!   [`NvmWriter`](crate::io::NvmWriter).

#[cfg(feature = "avr128")]
pub use self::avr128::{EepromState, FlashNvmWriter, UserRowKeyStore};
#[cfg(feature = "drivers")]
pub use self::cat25::Cat25Store;

#[cfg(feature = "avr128")]
mod avr128;
#[cfg(feature = "drivers")]
mod cat25;
