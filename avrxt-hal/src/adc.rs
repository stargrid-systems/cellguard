//! Single-conversion analog input on ADC0.
//!
//! [`Adc`] is generic over an [`AdcInstance`]. It uses the reset-default voltage
//! reference. Configure [`vref`](crate::vref) beforehand if a specific reference
//! is required. The per-family register differences (12/10-bit on AVR128 vs
//! 10/8-bit on tinyAVR) live in the `avr128` and `tiny` submodules.

#[cfg(feature = "_avr128")]
mod avr128;
#[cfg(feature = "_tinyavr")]
mod tiny;

/// ADC clock prescaler (`CTRLC.PRESC`).
#[derive(Clone, Copy)]
pub enum Prescaler {
    Div2,
    Div4,
    Div8,
    Div16,
    Div32,
    Div64,
    Div128,
    Div256,
}

/// ADC resolution (`CTRLA.RESSEL`). Each variant exists only on the families that
/// support it. Passing a variant to an ADC that cannot produce it panics.
#[derive(Clone, Copy)]
pub enum Resolution {
    /// AVR128 only.
    #[cfg(feature = "_avr128")]
    Bits12,
    Bits10,
    /// tinyAVR only.
    #[cfg(feature = "_tinyavr")]
    Bits8,
}

/// An ADC peripheral. Implemented for each device's `ADC0`. Not for external use.
pub trait AdcInstance {
    /// Enables the ADC in single-conversion mode.
    fn configure(&self, prescaler: Prescaler, resolution: Resolution);
    /// Sets the sampling length (`SAMPCTRL.SAMPLEN`) in ADC clock cycles. A longer
    /// sample helps high-impedance sources such as the temperature sensor.
    fn set_sample_length(&self, cycles: u8);
    /// Runs one conversion on the given positive-mux channel and returns the raw
    /// result.
    fn convert(&self, channel: u8) -> u16;
}

/// Single-ended ADC.
pub struct Adc<T: AdcInstance> {
    instance: T,
}

impl<T: AdcInstance> Adc<T> {
    /// Enables the ADC. Writes `CTRLA`/`CTRLC` (reset then configure).
    ///
    /// # Panics
    /// Panics if `resolution` is one this device's ADC cannot produce (for
    /// example `Bits12` on tinyAVR or `Bits8` on AVR128).
    #[must_use]
    pub fn new(instance: T, prescaler: Prescaler, resolution: Resolution) -> Self {
        instance.configure(prescaler, resolution);
        Self { instance }
    }

    /// Performs one conversion on the given `AINxx` channel index.
    pub fn read_channel(&mut self, channel: u8) -> u16 {
        self.instance.convert(channel)
    }

    /// Sets the sampling length in ADC clock cycles. `SAMPLEN` is a 5-bit field.
    ///
    /// # Panics
    /// Panics if `cycles` exceeds 31, rather than silently truncating it.
    pub fn set_sample_length(&mut self, cycles: u8) {
        assert!(cycles <= 31, "ADC sample length must be 0..=31");
        self.instance.set_sample_length(cycles);
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}
