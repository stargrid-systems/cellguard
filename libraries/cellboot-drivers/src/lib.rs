//! Adapters that implement `cellboot`'s I/O traits over concrete device drivers.
//!
//! These bridge the hardware-agnostic driver crates (like `cat25`) to the traits
//! in `cellboot::io`, so a firmware binary can hand `cellboot` a real backing
//! store without re-implementing the trait itself (the orphan rule needs the
//! adapter to live in a crate that owns either the trait or the type, so each
//! adapter is a local newtype here).
//!
//! # Features
//!
//! Features are additive and off by default.
//!
//! - `avr128`: on-chip NVM adapters ([`EepromState`], [`UserRowKeyStore`])
//!   backed by `avrxt-hal`. This pulls in the HAL, so it only builds for the
//!   `avr-none` target.
#![no_std]
#![warn(missing_docs)]

#[cfg(feature = "avr128")]
pub use self::avr128::{EepromState, UserRowKeyStore};

#[cfg(feature = "avr128")]
mod avr128;

use cat25::{Cat25, Error};
use cellboot::io::ImageStore;
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiDevice;

/// A `cellboot` [`ImageStore`] backed by a CAT25 SPI EEPROM.
///
/// The staged firmware image lives in this EEPROM: the AVR128 writes it and the
/// `cellprog` programmer reads it back.
pub struct Cat25Store<S, D>(Cat25<S, D>);

impl<S, D> Cat25Store<S, D> {
    /// Wraps a CAT25 driver as an image store.
    #[must_use]
    pub const fn new(driver: Cat25<S, D>) -> Self {
        Self(driver)
    }

    /// Returns the wrapped driver.
    #[must_use]
    pub fn into_inner(self) -> Cat25<S, D> {
        self.0
    }
}

impl<S: SpiDevice, D: DelayNs> ImageStore for Cat25Store<S, D> {
    type Error = Error<S::Error>;

    fn capacity(&self) -> u32 {
        self.0.model().size()
    }

    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.0.read(offset, buf)
    }

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.0.write(offset, data)
    }
}

#[cfg(test)]
mod tests {
    use cat25::{CAT25128, CAT25M01, Cat25};
    use cellboot::io::ImageStore;
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::spi::Mock as SpiMock;

    use super::Cat25Store;

    #[test]
    fn reports_model_capacity() {
        let mut spi = SpiMock::new(&[]);
        let store = Cat25Store::new(Cat25::new(spi.clone(), CAT25128, NoopDelay));
        assert_eq!(store.capacity(), 16 * 1024);
        spi.done();

        let mut spi = SpiMock::new(&[]);
        let store = Cat25Store::new(Cat25::new(spi.clone(), CAT25M01, NoopDelay));
        assert_eq!(store.capacity(), 128 * 1024);
        spi.done();
    }
}
