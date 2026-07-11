//! SPI host bus implementing [`embedded_hal::spi::SpiBus`].
//!
//! [`Spi`] is generic over an [`SpiInstance`]. This is the bus only. Combine it
//! with a chip-select [`OutputPin`] using `embedded-hal-bus`
//! (`ExclusiveDevice`) to obtain a `SpiDevice`. Pin direction and `PORTMUX`
//! routing are the application's responsibility.
//!
//! [`OutputPin`]: embedded_hal::digital::OutputPin

use embedded_hal::spi::{self, Mode, SpiBus};
#[cfg(feature = "_avr128")]
use embedded_hal::spi::{Phase, Polarity};

/// SPI clock prescaler (divides `CLK_PER`).
#[derive(Clone, Copy)]
pub enum Prescaler {
    Div4,
    Div16,
    Div64,
    Div128,
}

/// SPI bus error. Host byte transfers on this peripheral cannot fail.
#[derive(Debug, Clone, Copy)]
pub enum Error {}

impl spi::Error for Error {
    #[expect(
        clippy::uninhabited_references,
        reason = "required `Error::kind` impl; `Error` is uninhabited"
    )]
    fn kind(&self) -> spi::ErrorKind {
        match *self {}
    }
}

/// A peripheral usable as an SPI host. Implemented for each device's
/// `SPI0`/`SPI1`. Not for external use.
pub trait SpiInstance {
    /// Enables the peripheral as an MSB-first host in the given mode/prescaler.
    fn configure(&self, mode: Mode, prescaler: Prescaler);
    /// Full-duplex transfer of one byte (blocking).
    fn transfer_byte(&self, byte: u8) -> u8;
}

/// SPI host bus built on an [`SpiInstance`].
pub struct Spi<T: SpiInstance> {
    instance: T,
}

impl<T: SpiInstance> Spi<T> {
    /// Enables the host in the given SPI mode and prescaler. Writes
    /// `CTRLA`/`CTRLB` whole (reset then configure).
    #[must_use]
    pub fn new(instance: T, mode: Mode, prescaler: Prescaler) -> Self {
        instance.configure(mode, prescaler);
        Self { instance }
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}

impl<T: SpiInstance> spi::ErrorType for Spi<T> {
    type Error = Error;
}

impl<T: SpiInstance> SpiBus<u8> for Spi<T> {
    fn read(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words {
            *w = self.instance.transfer_byte(0);
        }
        Ok(())
    }

    fn write(&mut self, words: &[u8]) -> Result<(), Self::Error> {
        for &w in words {
            let _ = self.instance.transfer_byte(w);
        }
        Ok(())
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]) -> Result<(), Self::Error> {
        let n = read.len().max(write.len());
        for i in 0..n {
            let out = write.get(i).copied().unwrap_or(0);
            let v = self.instance.transfer_byte(out);
            if let Some(slot) = read.get_mut(i) {
                *slot = v;
            }
        }
        Ok(())
    }

    fn transfer_in_place(&mut self, words: &mut [u8]) -> Result<(), Self::Error> {
        for w in words {
            *w = self.instance.transfer_byte(*w);
        }
        Ok(())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

// Hidden implementation detail. The bodies are identical across the distinct
// PAC register types. This private macro only emits trait impls, not types.
macro_rules! impl_spi_instance {
    ($SPI:ty) => {
        impl SpiInstance for $SPI {
            fn configure(&self, mode: Mode, prescaler: Prescaler) {
                // SSD=1 so driving the SS pin can't force the host into client mode.
                self.ctrlb().write(|w| {
                    w.ssd().set_bit();
                    match (mode.polarity, mode.phase) {
                        (Polarity::IdleLow, Phase::CaptureOnFirstTransition) => w.mode()._0(),
                        (Polarity::IdleLow, Phase::CaptureOnSecondTransition) => w.mode()._1(),
                        (Polarity::IdleHigh, Phase::CaptureOnFirstTransition) => w.mode()._2(),
                        (Polarity::IdleHigh, Phase::CaptureOnSecondTransition) => w.mode()._3(),
                    }
                });
                self.ctrla().write(|w| {
                    w.master().set_bit().enable().set_bit();
                    match prescaler {
                        Prescaler::Div4 => w.presc().div4(),
                        Prescaler::Div16 => w.presc().div16(),
                        Prescaler::Div64 => w.presc().div64(),
                        Prescaler::Div128 => w.presc().div128(),
                    }
                });
            }
            fn transfer_byte(&self, byte: u8) -> u8 {
                self.data().write(|w| w.set(byte));
                crate::wait::spin_until(|| self.intflags().read().if_().bit_is_set());
                self.data().read().bits()
            }
        }
    };
}

// One call per device (grouped, so instances never interleave). All three have
// SPI0 and SPI1.
macro_rules! impl_spis {
    ($($SPI:ty),+ $(,)?) => {
        $( impl_spi_instance!($SPI); )+
    };
}

#[cfg(feature = "avr128db48")]
impl_spis!(avr_device::avr128db48::SPI0, avr_device::avr128db48::SPI1);
#[cfg(feature = "avr128db64")]
impl_spis!(avr_device::avr128db64::SPI0, avr_device::avr128db64::SPI1);
#[cfg(feature = "avr128da64")]
impl_spis!(avr_device::avr128da64::SPI0, avr_device::avr128da64::SPI1);
