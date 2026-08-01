//! AVR128 high-frequency clock control.
//!
//! The AVR128 DB/DA family boots on the internal OSCHF at 4 MHz. [`set_oschf`]
//! selects another OSCHF frequency. [`set_extclk`] switches to an external
//! clock.

use super::{CcpUnlock, impl_ccp_unlock};

/// Internal high-frequency oscillator (OSCHF) frequency options.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum HfFreq {
    Mhz1,
    Mhz2,
    Mhz3,
    Mhz4,
    Mhz8,
    Mhz12,
    Mhz16,
    Mhz20,
    Mhz24,
}

impl HfFreq {
    /// The selected frequency in Hz (e.g. for [`Delay`](crate::delay::Delay)).
    #[must_use]
    pub const fn hz(self) -> u32 {
        match self {
            Self::Mhz1 => 1_000_000,
            Self::Mhz2 => 2_000_000,
            Self::Mhz3 => 3_000_000,
            Self::Mhz4 => 4_000_000,
            Self::Mhz8 => 8_000_000,
            Self::Mhz12 => 12_000_000,
            Self::Mhz16 => 16_000_000,
            Self::Mhz20 => 20_000_000,
            Self::Mhz24 => 24_000_000,
        }
    }
}

/// Controls the high-frequency oscillator. Implemented for each device's
/// `CLKCTRL`. Not for external use.
pub trait OscControl {
    /// Writes `OSCHFCTRLA.FRQSEL`. This register is protected, so the caller
    /// must have just called [`CcpUnlock::unlock_ioreg`].
    fn write_frqsel(&self, freq: HfFreq);
    /// Whether the high-frequency oscillator reports stable.
    fn oschf_stable(&self) -> bool;
}

/// Selects the internal OSCHF frequency and waits for it to stabilize.
///
/// # Panics
/// Panics if the oscillator does not report stable within the defensive
/// spin budget.
#[inline]
pub fn set_oschf<C: CcpUnlock, K: OscControl>(cpu: &C, clkctrl: &K, freq: HfFreq) {
    avr_device::interrupt::free(|_| {
        cpu.unlock_ioreg();
        clkctrl.write_frqsel(freq);
    });
    crate::wait::spin_until(|| clkctrl.oschf_stable());
}

macro_rules! impl_osc_control {
    ($CLKCTRL:ty) => {
        impl OscControl for $CLKCTRL {
            #[inline(always)]
            fn write_frqsel(&self, freq: HfFreq) {
                self.oschfctrla().write(|w| match freq {
                    HfFreq::Mhz1 => w.frqsel()._1m(),
                    HfFreq::Mhz2 => w.frqsel()._2m(),
                    HfFreq::Mhz3 => w.frqsel()._3m(),
                    HfFreq::Mhz4 => w.frqsel()._4m(),
                    HfFreq::Mhz8 => w.frqsel()._8m(),
                    HfFreq::Mhz12 => w.frqsel()._12m(),
                    HfFreq::Mhz16 => w.frqsel()._16m(),
                    HfFreq::Mhz20 => w.frqsel()._20m(),
                    HfFreq::Mhz24 => w.frqsel()._24m(),
                });
            }
            fn oschf_stable(&self) -> bool {
                self.mclkstatus().read().oschfs().bit_is_set()
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_osc_control!(avr_device::avr128db48::CLKCTRL);
#[cfg(feature = "avr128db64")]
impl_osc_control!(avr_device::avr128db64::CLKCTRL);
#[cfg(feature = "avr128da64")]
impl_osc_control!(avr_device::avr128da64::CLKCTRL);

/// External clock source control. Implemented for each device's `CLKCTRL`. Not
/// for external use.
///
/// DA parts take the clock directly from the EXTCLK pin. DB parts route it
/// through `XOSCHF`, which must be configured and enabled first.
pub trait ExtClockControl {
    /// Configures and enables the external clock source. No-op on DA. Caller
    /// must have just called [`CcpUnlock::unlock_ioreg`].
    fn enable_extclk(&self, freq: HfFreq);
    /// Selects the external clock as the main clock. Caller must have just
    /// called [`CcpUnlock::unlock_ioreg`].
    fn select_extclk(&self);
    /// Whether the clock switch is still in progress.
    fn switch_in_progress(&self) -> bool;
}

/// Switches the main clock to an external clock and waits for the switch to
/// complete.
///
/// `freq` picks the `XOSCHF.FRQRANGE` on DB parts and is the frequency the
/// caller should feed to [`Delay`](crate::delay::Delay). Limited to standard
/// OSCHF steps. Ignored on DA parts.
///
/// # Panics
/// Panics if the switch does not complete within the defensive spin budget.
#[inline]
pub fn set_extclk<C: CcpUnlock, K: ExtClockControl>(cpu: &C, clkctrl: &K, freq: HfFreq) {
    avr_device::interrupt::free(|_| {
        cpu.unlock_ioreg();
        clkctrl.enable_extclk(freq);
        cpu.unlock_ioreg();
        clkctrl.select_extclk();
    });
    crate::wait::spin_until(|| !clkctrl.switch_in_progress());
}

// DB parts: external clock routes through XOSCHF.
macro_rules! impl_extclk_xoschf {
    ($CLKCTRL:ty) => {
        impl ExtClockControl for $CLKCTRL {
            #[inline(always)]
            fn enable_extclk(&self, freq: HfFreq) {
                self.xoschfctrla().write(|w| {
                    w.selhf().extclock();
                    match freq {
                        HfFreq::Mhz1
                        | HfFreq::Mhz2
                        | HfFreq::Mhz3
                        | HfFreq::Mhz4
                        | HfFreq::Mhz8 => w.frqrange()._8m(),
                        HfFreq::Mhz12 | HfFreq::Mhz16 => w.frqrange()._16m(),
                        HfFreq::Mhz20 | HfFreq::Mhz24 => w.frqrange()._24m(),
                    };
                    w.enable().set_bit()
                });
            }
            #[inline(always)]
            fn select_extclk(&self) {
                self.mclkctrla().write(|w| w.clksel().extclk());
            }
            fn switch_in_progress(&self) -> bool {
                self.mclkstatus().read().sosc().bit_is_set()
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_extclk_xoschf!(avr_device::avr128db48::CLKCTRL);
#[cfg(feature = "avr128db64")]
impl_extclk_xoschf!(avr_device::avr128db64::CLKCTRL);
#[cfg(feature = "avr128da64")]
impl ExtClockControl for avr_device::avr128da64::CLKCTRL {
    fn enable_extclk(&self, _freq: HfFreq) {}
    fn select_extclk(&self) {
        self.mclkctrla().write(|w| w.clksel().extclk());
    }
    fn switch_in_progress(&self) -> bool {
        self.mclkstatus().read().sosc().bit_is_set()
    }
}

#[cfg(feature = "avr128db48")]
impl_ccp_unlock!(avr_device::avr128db48::CPU);
#[cfg(feature = "avr128db64")]
impl_ccp_unlock!(avr_device::avr128db64::CPU);
#[cfg(feature = "avr128da64")]
impl_ccp_unlock!(avr_device::avr128da64::CPU);
