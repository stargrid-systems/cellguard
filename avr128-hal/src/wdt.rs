//! Watchdog timer (WDT).
//!
//! [`Watchdog`] is generic over a [`WdtInstance`]. The WDT runs from the
//! internal 1.024 kHz oscillator, so `Clk1k` is about 1 second. `CTRLA` is
//! configuration-change protected. [`Watchdog::start`] takes a
//! [`CcpUnlock`] (the device's `CPU`) and unlocks CCP right before the inlined
//! protected write.

use crate::clock::CcpUnlock;

/// Watchdog time-out, in WDT clock cycles (about 1.024 kHz).
#[derive(Clone, Copy)]
pub enum Period {
    /// Watchdog disabled.
    Off,
    Clk8,
    Clk16,
    Clk32,
    Clk64,
    Clk128,
    Clk256,
    Clk512,
    /// About 1 second.
    Clk1k,
    /// About 2 seconds.
    Clk2k,
    /// About 4 seconds.
    Clk4k,
    /// About 8 seconds.
    Clk8k,
}

/// A WDT peripheral. Implemented for each device's `WDT`. Not for external use.
pub trait WdtInstance {
    /// Writes `CTRLA.PERIOD`. This is protected, so the caller must unlock CCP
    /// just before.
    fn write_period(&self, period: Period);
}

/// The watchdog timer.
pub struct Watchdog<T: WdtInstance> {
    _instance: T,
}

impl<T: WdtInstance> Watchdog<T> {
    /// Starts (or, with [`Period::Off`], disables) the watchdog in normal mode.
    #[inline(always)]
    #[must_use]
    pub fn start<C: CcpUnlock>(cpu: &C, instance: T, period: Period) -> Self {
        cpu.unlock_ioreg();
        instance.write_period(period);
        Self {
            _instance: instance,
        }
    }

    /// Resets the watchdog count (issues `WDR`). Call within the configured
    /// period to prevent a reset.
    #[inline]
    pub fn feed(&mut self) {
        avr_device::asm::wdr();
    }
}

macro_rules! impl_wdt_instance {
    ($WDT:ty) => {
        impl WdtInstance for $WDT {
            #[inline(always)]
            fn write_period(&self, period: Period) {
                self.ctrla().write(|w| match period {
                    Period::Off => w.period().off(),
                    Period::Clk8 => w.period()._8clk(),
                    Period::Clk16 => w.period()._16clk(),
                    Period::Clk32 => w.period()._32clk(),
                    Period::Clk64 => w.period()._64clk(),
                    Period::Clk128 => w.period()._128clk(),
                    Period::Clk256 => w.period()._256clk(),
                    Period::Clk512 => w.period()._512clk(),
                    Period::Clk1k => w.period()._1kclk(),
                    Period::Clk2k => w.period()._2kclk(),
                    Period::Clk4k => w.period()._4kclk(),
                    Period::Clk8k => w.period()._8kclk(),
                });
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_wdt_instance!(avr_device::avr128db48::WDT);
#[cfg(feature = "avr128db64")]
impl_wdt_instance!(avr_device::avr128db64::WDT);
#[cfg(feature = "avr128da64")]
impl_wdt_instance!(avr_device::avr128da64::WDT);
