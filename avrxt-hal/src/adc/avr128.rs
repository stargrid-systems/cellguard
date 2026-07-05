//! AVR128 ADC0 (12/10-bit).

use super::{AdcInstance, Avr128Resolution, Prescaler};

macro_rules! impl_adc_instance {
    ($ADC:ty) => {
        impl AdcInstance for $ADC {
            type Resolution = Avr128Resolution;
            fn configure(&self, prescaler: Prescaler, resolution: Avr128Resolution) {
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
                        Avr128Resolution::Bits12 => w.ressel()._12bit(),
                        Avr128Resolution::Bits10 => w.ressel()._10bit(),
                    };
                    w.enable().set_bit()
                });
            }
            fn set_sample_length(&self, cycles: u8) {
                self.sampctrl().write(|w|
                    // SAFETY: SAMPLEN is a 5-bit field. The caller
                    // (Adc::set_sample_length) has already checked the range.
                    unsafe { w.samplen().bits(cycles) });
            }
            fn convert(&self, channel: u8) -> u16 {
                self.muxpos().write(|w|
                    // SAFETY: `channel` selects an analog input. Out-of-range values
                    // read an unconnected or ground input, never UB.
                    unsafe { w.muxpos().bits(channel) });
                self.command().write(|w| w.stconv().set_bit());
                crate::wait::spin_until(|| self.intflags().read().resrdy().bit_is_set());
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
