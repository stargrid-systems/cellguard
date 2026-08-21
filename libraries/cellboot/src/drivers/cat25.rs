//! An [`ImageStore`] backed by a CAT25 SPI EEPROM.

use cat25::{Cat25, Error};
use embedded_hal::delay::DelayNs;
use embedded_hal::spi::SpiDevice;

use crate::io::ImageStore;

/// An [`ImageStore`] backed by a CAT25 SPI EEPROM.
///
/// The AVR128 stages the image here and the PROG MCU reads it back.
pub struct Cat25Store<S, D>(Cat25<S, D>);

impl<S, D> Cat25Store<S, D> {
    /// Wraps a CAT25 driver as an image store.
    #[must_use]
    pub const fn new(driver: Cat25<S, D>) -> Self {
        Self(driver)
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
    use cat25::{CAT25M01, CAT25128, Cat25};
    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::spi::Mock as SpiMock;

    use super::Cat25Store;
    use crate::io::ImageStore;

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
