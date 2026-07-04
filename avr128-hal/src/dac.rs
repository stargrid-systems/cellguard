//! 10-bit analog output on DAC0 (drives the DAC0 pin, PD6).
//!
//! [`Dac`] is generic over a [`DacInstance`]. It uses the reset-default DAC
//! reference. Set [`vref`](crate::vref) beforehand for a specific reference.

/// A DAC peripheral. Implemented for each device's `DAC0`. Not for external
/// use.
pub trait DacInstance {
    /// Enables the DAC with its output buffer routed to the pin.
    fn enable(&self);
    /// Writes the raw 16-bit (left-justified) data register.
    fn write_data(&self, value: u16);
}

/// 10-bit DAC with the output buffer enabled.
pub struct Dac<T: DacInstance> {
    instance: T,
}

impl<T: DacInstance> Dac<T> {
    /// Enables the DAC and routes it to the output pin. Writes `CTRLA` whole
    /// (reset then configure).
    #[must_use]
    pub fn new(instance: T) -> Self {
        instance.enable();
        Self { instance }
    }

    /// Sets the 10-bit output value (only the low 10 bits are used).
    pub fn set(&mut self, value: u16) {
        // DATA holds the value left-justified (bits 15:6).
        self.instance.write_data((value & 0x03FF) << 6);
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}

macro_rules! impl_dac_instance {
    ($DAC:ty) => {
        impl DacInstance for $DAC {
            fn enable(&self) {
                self.ctrla()
                    .write(|w| w.enable().set_bit().outen().set_bit());
            }
            fn write_data(&self, value: u16) {
                self.data().write(|w| w.data().set(value));
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_dac_instance!(avr_device::avr128db48::DAC0);
#[cfg(feature = "avr128db64")]
impl_dac_instance!(avr_device::avr128db64::DAC0);
#[cfg(feature = "avr128da64")]
impl_dac_instance!(avr_device::avr128da64::DAC0);
