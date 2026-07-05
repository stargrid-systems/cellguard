//! AVR128 high-frequency oscillator (OSCHF) clock control.
//!
//! The AVR128 DB/DA family boots on OSCHF at 4 MHz. [`set_oschf`] selects another
//! OSCHF frequency. `OSCHFCTRLA` is configuration-change protected.

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
    /// Writes `OSCHFCTRLA.FRQSEL`. This register is protected, so the caller must
    /// have just called [`CcpUnlock::unlock_ioreg`].
    fn write_frqsel(&self, freq: HfFreq);
    /// Whether the high-frequency oscillator reports stable.
    fn oschf_stable(&self) -> bool;
}

/// Selects the internal high-frequency oscillator frequency and waits for it to
/// stabilize. The main clock source stays OSCHF (the reset default) with no
/// prescaler, so `CLK_PER` becomes `freq`.
///
/// `OSCHFCTRLA` is written whole (reset then configure). The CCP unlock and the
/// protected write happen with interrupts masked so the unlock window cannot be
/// interrupted.
///
/// # Panics
/// Panics if the oscillator does not report stable within the defensive
/// spin budget, which means the peripheral is broken or misconfigured.
#[inline(always)]
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

#[cfg(feature = "avr128db48")]
impl_ccp_unlock!(avr_device::avr128db48::CPU);
#[cfg(feature = "avr128db64")]
impl_ccp_unlock!(avr_device::avr128db64::CPU);
#[cfg(feature = "avr128da64")]
impl_ccp_unlock!(avr_device::avr128da64::CPU);
