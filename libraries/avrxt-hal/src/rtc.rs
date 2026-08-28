//! Real-time counter (RTC). A free-running 16-bit counter with a programmable
//! period, useful as a periodic time base.
//!
//! [`Rtc`] is generic over an [`RtcInstance`]. RTC register writes are
//! synchronised to the RTC clock domain. This driver waits for the relevant
//! busy flag before each write.

/// RTC clock source (`CLKSEL`).
#[derive(Clone, Copy)]
pub enum ClockSource {
    /// Internal 32.768 kHz ultra-low-power oscillator.
    Internal32k,
    /// Internal 1.024 kHz (32.768 kHz / 32) ultra-low-power oscillator.
    Internal1k,
    /// External 32.768 kHz crystal.
    External32k,
    /// External clock on the EXTCLK pin.
    ExternalClock,
}

impl ClockSource {
    const fn code(self) -> u8 {
        match self {
            Self::Internal32k => 0,
            Self::Internal1k => 1,
            Self::External32k => 2,
            Self::ExternalClock => 3,
        }
    }
}

/// RTC prescaler (`CTRLA.PRESCALER`): divides the clock source.
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
    /// Divide by 32.
    Div32,
    /// Divide by 64.
    Div64,
    /// Divide by 128.
    Div128,
    /// Divide by 256.
    Div256,
    /// Divide by 512.
    Div512,
    /// Divide by 1024.
    Div1024,
    /// Divide by 2048.
    Div2048,
    /// Divide by 4096.
    Div4096,
    /// Divide by 8192.
    Div8192,
    /// Divide by 16384.
    Div16384,
    /// Divide by 32768.
    Div32768,
}

impl Prescaler {
    const fn code(self) -> u8 {
        self as u8
    }
}

/// An RTC peripheral. Implemented for each device's `RTC`. Not for external
/// use.
pub trait RtcInstance {
    /// Enables the RTC counting from `0` up to `period`, then wrapping.
    fn configure(&self, source_code: u8, prescaler_code: u8, period: u16);
    /// Reads the current counter value.
    fn count(&self) -> u16;
}

/// The real-time counter.
pub struct Rtc<T: RtcInstance> {
    instance: T,
}

impl<T: RtcInstance> Rtc<T> {
    /// Enables the RTC counting from `0` up to `period`, then wrapping. Writes
    /// `CLKSEL`/`PER`/`CTRLA` whole (reset then configure).
    #[must_use]
    pub fn new(instance: T, source: ClockSource, prescaler: Prescaler, period: u16) -> Self {
        instance.configure(source.code(), prescaler.code(), period);
        Self { instance }
    }

    /// Reads the current counter value.
    pub fn count(&self) -> u16 {
        self.instance.count()
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}

macro_rules! impl_rtc_instance {
    ($RTC:ty) => {
        impl RtcInstance for $RTC {
            fn configure(&self, source_code: u8, prescaler_code: u8, period: u16) {
                self.clksel().write(|w|
                    // SAFETY: `source_code` is a valid CLKSEL selection (0..=3).
                    unsafe { w.clksel().bits(source_code) });
                crate::wait::spin_until(|| self.status().read().perbusy().bit_is_clear());
                self.per().write(|w| w.set(period));
                crate::wait::spin_until(|| self.status().read().ctrlabusy().bit_is_clear());
                self.ctrla().write(|w| {
                    // SAFETY: `prescaler_code` is a valid PRESCALER selection (0..=15).
                    unsafe { w.prescaler().bits(prescaler_code) };
                    w.rtcen().set_bit()
                });
            }
            fn count(&self) -> u16 {
                self.cnt().read().bits()
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_rtc_instance!(avr_device::avr128db48::RTC);
#[cfg(feature = "avr128db64")]
impl_rtc_instance!(avr_device::avr128db64::RTC);
#[cfg(feature = "avr128da64")]
impl_rtc_instance!(avr_device::avr128da64::RTC);
#[cfg(feature = "attiny406")]
impl_rtc_instance!(avr_device::attiny406::RTC);
#[cfg(feature = "attiny416")]
impl_rtc_instance!(avr_device::attiny416::RTC);

#[cfg(test)]
mod tests {
    use super::{ClockSource, Prescaler};

    #[test]
    fn clock_source_codes() {
        assert_eq!(ClockSource::Internal32k.code(), 0);
        assert_eq!(ClockSource::Internal1k.code(), 1);
        assert_eq!(ClockSource::External32k.code(), 2);
        assert_eq!(ClockSource::ExternalClock.code(), 3);
    }

    #[test]
    fn prescaler_codes() {
        let cases = [
            (Prescaler::Div1, 0),
            (Prescaler::Div2, 1),
            (Prescaler::Div4, 2),
            (Prescaler::Div8, 3),
            (Prescaler::Div16, 4),
            (Prescaler::Div32, 5),
            (Prescaler::Div64, 6),
            (Prescaler::Div128, 7),
            (Prescaler::Div256, 8),
            (Prescaler::Div512, 9),
            (Prescaler::Div1024, 10),
            (Prescaler::Div2048, 11),
            (Prescaler::Div4096, 12),
            (Prescaler::Div8192, 13),
            (Prescaler::Div16384, 14),
            (Prescaler::Div32768, 15),
        ];
        for (prescaler, code) in cases {
            assert_eq!(prescaler.code(), code);
        }
    }
}
