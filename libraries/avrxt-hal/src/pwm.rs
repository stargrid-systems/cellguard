//! Single-slope PWM on the 16-bit timer TCA0.
//!
//! [`Pwm`] is generic over a [`TcaInstance`]. TCA0 in single mode drives three
//! compare outputs (WO0/WO1/WO2). The PWM frequency is
//! `f_CLK_PER / (prescale * (PER + 1))`. `PORTMUX` and pin direction for the WO
//! outputs are the application's responsibility.

/// TCA0 clock prescaler (`CLKSEL`).
#[derive(Clone, Copy)]
pub enum Prescaler {
    /// Divide by 1.
    Div1,
    /// Divide by 2.
    Div2,
    /// Divide by 4.
    Div4,
    /// Divide by 8.
    Div8,
    /// Divide by 16.
    Div16,
    /// Divide by 64.
    Div64,
    /// Divide by 256.
    Div256,
    /// Divide by 1024.
    Div1024,
}

/// One of the three single-mode compare channels.
#[derive(Clone, Copy)]
pub enum Channel {
    /// Compare channel WO0.
    Wo0,
    /// Compare channel WO1.
    Wo1,
    /// Compare channel WO2.
    Wo2,
}

impl Channel {
    const fn index(self) -> u8 {
        match self {
            Self::Wo0 => 0,
            Self::Wo1 => 1,
            Self::Wo2 => 2,
        }
    }
}

/// A TCA timer usable for PWM. Implemented for each device's `TCA0`. Not for
/// external use.
pub trait TcaInstance {
    /// Configures single-slope PWM with the given period and prescaler, enables
    /// all three compare outputs (at 0% duty), and starts the timer.
    fn configure(&self, period: u16, prescaler: Prescaler);
    /// Sets the compare value (duty in ticks) for channel `ch` (0..=2).
    fn set_compare(&self, ch: u8, duty: u16);
}

/// Single-slope PWM generator on a TCA timer.
pub struct Pwm<T: TcaInstance> {
    instance: T,
}

impl<T: TcaInstance> Pwm<T> {
    /// Configures and starts single-slope PWM. Writes the `SINGLE_*` control
    /// registers whole (reset then configure).
    #[must_use]
    pub fn new(instance: T, period: u16, prescaler: Prescaler) -> Self {
        instance.configure(period, prescaler);
        Self { instance }
    }

    /// Sets the compare value (duty, in timer ticks) for one channel.
    pub fn set_duty(&mut self, channel: Channel, duty: u16) {
        self.instance.set_compare(channel.index(), duty);
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}

macro_rules! impl_tca_instance {
    ($TCA:ty) => {
        impl TcaInstance for $TCA {
            fn configure(&self, period: u16, prescaler: Prescaler) {
                self.single_ctrlb().write(|w| {
                    w.cmp0en().set_bit().cmp1en().set_bit().cmp2en().set_bit();
                    w.wgmode().singleslope()
                });
                self.single_per().write(|w| w.set(period));
                self.single_cmp0().write(|w| w.set(0));
                self.single_cmp1().write(|w| w.set(0));
                self.single_cmp2().write(|w| w.set(0));
                self.single_ctrla().write(|w| {
                    match prescaler {
                        Prescaler::Div1 => w.clksel().div1(),
                        Prescaler::Div2 => w.clksel().div2(),
                        Prescaler::Div4 => w.clksel().div4(),
                        Prescaler::Div8 => w.clksel().div8(),
                        Prescaler::Div16 => w.clksel().div16(),
                        Prescaler::Div64 => w.clksel().div64(),
                        Prescaler::Div256 => w.clksel().div256(),
                        Prescaler::Div1024 => w.clksel().div1024(),
                    };
                    w.enable().set_bit()
                });
            }
            fn set_compare(&self, ch: u8, duty: u16) {
                match ch {
                    0 => self.single_cmp0().write(|w| w.set(duty)),
                    1 => self.single_cmp1().write(|w| w.set(duty)),
                    _ => self.single_cmp2().write(|w| w.set(duty)),
                };
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_tca_instance!(avr_device::avr128db48::TCA0);
#[cfg(feature = "avr128db64")]
impl_tca_instance!(avr_device::avr128db64::TCA0);
#[cfg(feature = "avr128da64")]
impl_tca_instance!(avr_device::avr128da64::TCA0);

#[cfg(test)]
mod tests {
    use super::Channel;

    #[test]
    fn channel_indices() {
        assert_eq!(Channel::Wo0.index(), 0);
        assert_eq!(Channel::Wo1.index(), 1);
        assert_eq!(Channel::Wo2.index(), 2);
    }
}
