//! Watchdog timer (WDT).
//!
//! [`Watchdog`] is generic over a [`WdtInstance`]. The WDT runs from the
//! internal 1.024 kHz oscillator, so `Clk1k` is about 1 second. `CTRLA` is
//! configuration-change protected. [`Watchdog::start`] takes a
//! [`CcpUnlock`] (the device's `CPU`) and does the CCP unlock plus the
//! protected write with interrupts masked, so the unlock window cannot be
//! interrupted.
//!
//! [`Watchdog::start`] and [`Watchdog::stop`] first wait out
//! `STATUS.SYNCBUSY`. The hardware ignores a `CTRLA` write while the
//! previous one is still syncing into the WDT clock domain.

use crate::clock::CcpUnlock;

/// Watchdog time-out, in WDT clock cycles (about 1.024 kHz).
#[derive(Clone, Copy)]
pub enum Period {
    /// Watchdog disabled.
    Off,
    /// About 8 ms.
    Clk8,
    /// About 16 ms.
    Clk16,
    /// About 31 ms.
    Clk32,
    /// About 63 ms.
    Clk64,
    /// About 125 ms.
    Clk128,
    /// About 250 ms.
    Clk256,
    /// About 500 ms.
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
    /// Spins until `STATUS.SYNCBUSY` is clear. A `CTRLA` write takes 2 to 3
    /// WDT clock cycles (about 3 ms) to cross into the WDT clock domain, and
    /// the hardware silently ignores `CTRLA` writes while that is pending.
    fn wait_sync(&self);
}

/// The watchdog timer.
pub struct Watchdog<T: WdtInstance> {
    instance: T,
}

impl<T: WdtInstance> Watchdog<T> {
    /// Starts (or, with [`Period::Off`], disables) the watchdog in normal mode.
    #[inline]
    #[must_use]
    pub fn start<C: CcpUnlock>(cpu: &C, instance: T, period: Period) -> Self {
        // Before the unlock: the CCP window is only 4 instructions.
        instance.wait_sync();
        avr_device::interrupt::free(|_| {
            cpu.unlock_ioreg();
            instance.write_period(period);
        });
        Self { instance }
    }

    /// Resets the watchdog count (issues `WDR`). Call within the configured
    /// period to prevent a reset.
    #[inline]
    pub fn feed(&mut self) {
        avr_device::asm::wdr();
    }

    /// Disables the watchdog. Consumes the handle so the stopped watchdog can
    /// no longer be fed.
    pub fn stop<C: CcpUnlock>(self, cpu: &C) {
        self.instance.wait_sync();
        avr_device::interrupt::free(|_| {
            cpu.unlock_ioreg();
            self.instance.write_period(Period::Off);
        });
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
            fn wait_sync(&self) {
                crate::wait::spin_until(|| self.status().read().syncbusy().bit_is_clear());
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
#[cfg(feature = "attiny406")]
impl_wdt_instance!(avr_device::attiny406::WDT);
#[cfg(feature = "attiny416")]
impl_wdt_instance!(avr_device::attiny416::WDT);
