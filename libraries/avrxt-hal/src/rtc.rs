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
    Div1,
    Div2,
    Div4,
    Div8,
    Div16,
    Div32,
    Div64,
    Div128,
    Div256,
    Div512,
    Div1024,
    Div2048,
    Div4096,
    Div8192,
    Div16384,
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
