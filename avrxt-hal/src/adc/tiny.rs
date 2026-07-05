//! tinyAVR ADC0 (10/8-bit).
//!
//! Same register set as the AVR128 ADC, but the reference select (`CTRLC.REFSEL`)
//! shares the register with the prescaler, so `CTRLC` is written with `modify` to
//! keep a reference configured before `Adc::new`.

use super::{AdcInstance, Prescaler, Resolution};

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
                        #[cfg(feature = "_avr128")]
                        Resolution::Bits12 => panic!("Resolution::Bits12 is AVR128-only"),
                        Resolution::Bits10 => w.ressel()._10bit(),
                        Resolution::Bits8 => w.ressel()._8bit(),
                    };
                    w.enable().set_bit()
                });
            }
            fn set_sample_length(&self, cycles: u8) {
                self.sampctrl().write(|w|
                    // SAFETY: SAMPLEN is a 5-bit field; the caller (Adc::set_sample_length)
                    // has already checked the range.
                    unsafe { w.samplen().bits(cycles) });
            }
            fn convert(&self, channel: u8) -> u16 {
                self.muxpos().write(|w|
                    // SAFETY: `channel` selects an analog input. Out-of-range values
                    // read an unconnected or ground input, never UB.
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
