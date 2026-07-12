//! Voltage reference selection (VREF) for ADC0 and DAC0.
//!
//! [`Vref`] is generic over a [`VrefInstance`] (implemented for each device's
//! `VREF`). Set the reference *before* starting a conversion or output that
//! should use it (see [`adc`](crate::adc) and [`dac`](crate::dac)).

/// Reference voltage selection.
#[derive(Clone, Copy)]
pub enum Reference {
    /// 1.024 V internal reference.
    V1_024,
    /// 2.048 V internal reference.
    V2_048,
    /// 2.500 V internal reference.
    V2_500,
    /// 4.096 V internal reference.
    V4_096,
    /// VDD as reference.
    Vdd,
    /// External reference on the VREFA pin.
    External,
}

impl Reference {
    /// The raw `REFSEL` field code.
    const fn code(self) -> u8 {
        match self {
            Self::V1_024 => 0,
            Self::V2_048 => 1,
            Self::V4_096 => 2,
            Self::V2_500 => 3,
            Self::Vdd => 5,
            Self::External => 6,
        }
    }
}

/// A VREF peripheral. Implemented for each device's `VREF`. Not for external
/// use.
pub trait VrefInstance {
    /// Selects the ADC0 reference by raw `REFSEL` code.
    fn set_adc0_refsel(&self, code: u8);
    /// Selects the DAC0 reference by raw `REFSEL` code.
    fn set_dac0_refsel(&self, code: u8);
}

/// The voltage-reference peripheral.
pub struct Vref<T: VrefInstance> {
    instance: T,
}

impl<T: VrefInstance> Vref<T> {
    /// Takes ownership of the VREF peripheral.
    #[must_use]
    pub const fn new(instance: T) -> Self {
        Self { instance }
    }

    /// Selects the reference used by ADC0.
    pub fn set_adc0(&mut self, reference: Reference) {
        self.instance.set_adc0_refsel(reference.code());
    }

    /// Selects the reference used by DAC0.
    pub fn set_dac0(&mut self, reference: Reference) {
        self.instance.set_dac0_refsel(reference.code());
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}

macro_rules! impl_vref_instance {
    ($VREF:ty) => {
        impl VrefInstance for $VREF {
            fn set_adc0_refsel(&self, code: u8) {
                self.adc0ref().write(|w|
                    // SAFETY: `code` comes from `Reference::code`, always a
                    // valid REFSEL selection.
                    unsafe { w.refsel().bits(code) });
            }
            fn set_dac0_refsel(&self, code: u8) {
                self.dac0ref().write(|w|
                    // SAFETY: as above.
                    unsafe { w.refsel().bits(code) });
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_vref_instance!(avr_device::avr128db48::VREF);
#[cfg(feature = "avr128db64")]
impl_vref_instance!(avr_device::avr128db64::VREF);
#[cfg(feature = "avr128da64")]
impl_vref_instance!(avr_device::avr128da64::VREF);
