//! Single-conversion analog input on ADC0.
//!
//! [`Adc`] is generic over an [`AdcInstance`]. It uses the reset-default
//! voltage reference. Configure [`vref`](crate::vref) beforehand if a specific
//! reference is required.

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

/// ADC resolution (`CTRLA.RESSEL`). The AVR128 family supports 12/10-bit; the
/// tinyAVR ADC supports 10/8-bit. A resolution a device cannot produce is
/// clamped to that device's nearest one.
#[derive(Clone, Copy)]
pub enum Resolution {
    /// AVR128 only (clamped to 10-bit on tinyAVR).
    #[cfg(feature = "_avr128")]
    Bits12,
    Bits10,
    /// tinyAVR only (clamped to 10-bit on AVR128).
    #[cfg(feature = "_tinyavr")]
    Bits8,
}

/// An ADC peripheral. Implemented for each device's `ADC0`. Not for external
/// use.
pub trait AdcInstance {
    /// Enables the ADC in single-conversion mode.
    fn configure(&self, prescaler: Prescaler, resolution: Resolution);
    /// Sets the sampling length (`SAMPCTRL.SAMPLEN`) in ADC clock cycles. A
    /// longer sample helps high-impedance sources such as the temperature
    /// sensor.
    fn set_sample_length(&self, cycles: u8);
    /// Runs one conversion on the given positive-mux channel and returns the
    /// raw result.
    fn convert(&self, channel: u8) -> u16;
}

/// Single-ended ADC.
pub struct Adc<T: AdcInstance> {
    instance: T,
}

impl<T: AdcInstance> Adc<T> {
    /// Enables the ADC. Writes `CTRLA`/`CTRLC` whole (reset then configure).
    #[must_use]
    pub fn new(instance: T, prescaler: Prescaler, resolution: Resolution) -> Self {
        instance.configure(prescaler, resolution);
        Self { instance }
    }

    /// Performs one conversion on the given `AINxx` channel index.
    pub fn read_channel(&mut self, channel: u8) -> u16 {
        self.instance.convert(channel)
    }

    /// Sets the sampling length in ADC clock cycles (0..=31).
    pub fn set_sample_length(&mut self, cycles: u8) {
        self.instance.set_sample_length(cycles);
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}

macro_rules! impl_adc_instance {
    ($ADC:ty) => {
        impl AdcInstance for $ADC {
            fn configure(&self, prescaler: Prescaler, resolution: Resolution) {
                self.ctrlc().write(|w| match prescaler {
                    Prescaler::Div2 => w.presc().div2(),
                    Prescaler::Div4 => w.presc().div4(),
                    Prescaler::Div8 => w.presc().div8(),
                    Prescaler::Div16 => w.presc().div16(),
                    Prescaler::Div32 => w.presc().div32(),
                    Prescaler::Div64 => w.presc().div64(),
                    Prescaler::Div128 => w.presc().div128(),
                    Prescaler::Div256 => w.presc().div256(),
                });
                self.ctrla().write(|w| {
                    match resolution {
                        Resolution::Bits12 => w.ressel()._12bit(),
                        Resolution::Bits10 => w.ressel()._10bit(),
                        // AVR128 has no 8-bit mode; clamp to its minimum. This
                        // arm only exists when a tinyAVR feature is also on.
                        #[cfg(feature = "_tinyavr")]
                        Resolution::Bits8 => w.ressel()._10bit(),
                    };
                    w.enable().set_bit()
                });
            }
            fn set_sample_length(&self, cycles: u8) {
                self.sampctrl().write(|w|
                    // SAFETY: SAMPLEN is a plain 5-bit cycle count. Hardware
                    // ignores bits above the field width.
                    unsafe { w.samplen().bits(cycles) });
            }
            fn convert(&self, channel: u8) -> u16 {
                self.muxpos().write(|w|
                    // SAFETY: `channel` selects an analog input. Out-of-range
                    // values read an unconnected or ground input, never UB.
                    unsafe { w.muxpos().bits(channel) });
                self.command().write(|w| w.stconv().set_bit());
                while self.intflags().read().resrdy().bit_is_clear() {}
                self.res().read().bits()
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_adc_instance!(avr_device::avr128db48::ADC0);
#[cfg(feature = "avr128db64")]
impl_adc_instance!(avr_device::avr128db64::ADC0);
#[cfg(feature = "avr128da64")]
impl_adc_instance!(avr_device::avr128da64::ADC0);

// tinyAVR ADC0. Same register set as the AVR128 ADC, but 10/8-bit resolution
// and the reference select (`CTRLC.REFSEL`) lives in the same register as the
// prescaler. `CTRLC` is written with `modify` so a reference configured before
// `Adc::new` survives.
macro_rules! impl_adc_instance_tiny {
    ($ADC:ty) => {
        impl AdcInstance for $ADC {
            fn configure(&self, prescaler: Prescaler, resolution: Resolution) {
                self.ctrlc().modify(|_, w| match prescaler {
                    Prescaler::Div2 => w.presc().div2(),
                    Prescaler::Div4 => w.presc().div4(),
                    Prescaler::Div8 => w.presc().div8(),
                    Prescaler::Div16 => w.presc().div16(),
                    Prescaler::Div32 => w.presc().div32(),
                    Prescaler::Div64 => w.presc().div64(),
                    Prescaler::Div128 => w.presc().div128(),
                    Prescaler::Div256 => w.presc().div256(),
                });
                self.ctrla().write(|w| {
                    match resolution {
                        // tinyAVR has no 12-bit mode; clamp to its maximum. This
                        // arm only exists when an AVR128 feature is also on.
                        #[cfg(feature = "_avr128")]
                        Resolution::Bits12 => w.ressel()._10bit(),
                        Resolution::Bits10 => w.ressel()._10bit(),
                        Resolution::Bits8 => w.ressel()._8bit(),
                    };
                    w.enable().set_bit()
                });
            }
            fn set_sample_length(&self, cycles: u8) {
                self.sampctrl().write(|w|
                    // SAFETY: SAMPLEN is a plain cycle count; the field width is
                    // enforced by hardware.
                    unsafe { w.samplen().bits(cycles) });
            }
            fn convert(&self, channel: u8) -> u16 {
                self.muxpos().write(|w|
                    // SAFETY: `channel` selects an analog input. Out-of-range
                    // values read an unconnected or ground input, never UB.
                    unsafe { w.muxpos().bits(channel) });
                self.command().write(|w| w.stconv().set_bit());
                while self.intflags().read().resrdy().bit_is_clear() {}
                self.res().read().bits()
            }
        }
    };
}

#[cfg(feature = "attiny406")]
impl_adc_instance_tiny!(avr_device::attiny406::ADC0);
#[cfg(feature = "attiny416")]
impl_adc_instance_tiny!(avr_device::attiny416::ADC0);
