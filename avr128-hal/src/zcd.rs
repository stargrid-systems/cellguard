//! Zero-cross detector (ZCD).
//!
//! [`Zcd`] is generic over a [`ZcdInstance`]. The detector output can
//! optionally drive its pin. Its event output is always available to EVSYS/CCL.

/// A ZCD peripheral. Implemented for each device's `ZCD0`..`ZCD2`. Not for
/// external use.
pub trait ZcdInstance {
    /// Enables the detector, optionally driving its output pin.
    fn enable(&self, output_to_pin: bool);
    /// Disables the detector.
    fn disable(&self);
}

/// A zero-cross detector built on a [`ZcdInstance`].
pub struct Zcd<T: ZcdInstance> {
    instance: T,
}

impl<T: ZcdInstance> Zcd<T> {
    /// Enables the detector.
    #[must_use]
    pub fn new(instance: T, output_to_pin: bool) -> Self {
        instance.enable(output_to_pin);
        Self { instance }
    }

    /// Disables the detector and releases the peripheral.
    pub fn free(self) -> T {
        self.instance.disable();
        self.instance
    }
}

// Hidden implementation detail: trait impls only, no user-facing types.
macro_rules! impl_zcd_instance {
    ($ZCD:ty) => {
        impl ZcdInstance for $ZCD {
            fn enable(&self, output_to_pin: bool) {
                self.ctrla()
                    .write(|w| w.enable().set_bit().outen().bit(output_to_pin));
            }
            fn disable(&self) {
                self.ctrla().write(|w| w.enable().clear_bit());
            }
        }
    };
}

// db48/db64/da64 all have ZCD0..2.
#[cfg(feature = "avr128db48")]
impl_zcd_instance!(avr_device::avr128db48::ZCD0);
#[cfg(feature = "avr128db48")]
impl_zcd_instance!(avr_device::avr128db48::ZCD1);
#[cfg(feature = "avr128db48")]
impl_zcd_instance!(avr_device::avr128db48::ZCD2);
#[cfg(feature = "avr128db64")]
impl_zcd_instance!(avr_device::avr128db64::ZCD0);
#[cfg(feature = "avr128da64")]
impl_zcd_instance!(avr_device::avr128da64::ZCD0);
#[cfg(feature = "avr128db64")]
impl_zcd_instance!(avr_device::avr128db64::ZCD1);
#[cfg(feature = "avr128da64")]
impl_zcd_instance!(avr_device::avr128da64::ZCD1);
#[cfg(feature = "avr128db64")]
impl_zcd_instance!(avr_device::avr128db64::ZCD2);
#[cfg(feature = "avr128da64")]
impl_zcd_instance!(avr_device::avr128da64::ZCD2);
