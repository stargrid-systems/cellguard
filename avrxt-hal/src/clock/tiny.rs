//! tinyAVR main-clock control (`OSC20M` + prescaler).
//!
//! tinyAVR parts have no OSCHF. They boot on `OSC20M` (16 or 20 MHz, chosen by
//! the `OSCCFG` fuse) with the main-clock prescaler enabled at /6. Use
//! [`set_main_clock_prescaler`] to change or disable it. The base frequency is a
//! fuse setting, not runtime-changeable; pass it via [`TinyBaseFreq`] to compute
//! `CLK_PER`.

use super::CcpUnlock;

/// tinyAVR internal oscillator base frequency, selected by the `OSCCFG` fuse.
/// Not runtime-changeable; used only to compute `CLK_PER`.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TinyBaseFreq {
    Mhz16,
    Mhz20,
}

impl TinyBaseFreq {
    /// The base oscillator frequency in Hz.
    #[must_use]
    pub const fn hz(self) -> u32 {
        match self {
            Self::Mhz16 => 16_000_000,
            Self::Mhz20 => 20_000_000,
        }
    }

    /// The resulting `CLK_PER` in Hz for the given prescaler setting. `None` means
    /// the prescaler is disabled (`CLK_PER` == base frequency).
    #[must_use]
    pub const fn clk_per_hz(self, div: Option<ClkPrescaler>) -> u32 {
        match div {
            Some(d) => self.hz() / d.divisor(),
            None => self.hz(),
        }
    }
}

/// tinyAVR main-clock prescaler division (`MCLKCTRLB.PDIV`).
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum ClkPrescaler {
    Div2,
    Div4,
    Div6,
    Div8,
    Div10,
    Div12,
    Div16,
    Div24,
    Div32,
    Div48,
    Div64,
}

impl ClkPrescaler {
    /// The integer division factor.
    #[must_use]
    pub const fn divisor(self) -> u32 {
        match self {
            Self::Div2 => 2,
            Self::Div4 => 4,
            Self::Div6 => 6,
            Self::Div8 => 8,
            Self::Div10 => 10,
            Self::Div12 => 12,
            Self::Div16 => 16,
            Self::Div24 => 24,
            Self::Div32 => 32,
            Self::Div48 => 48,
            Self::Div64 => 64,
        }
    }
}

/// Controls the tinyAVR main-clock prescaler (`MCLKCTRLB`). Implemented for each
/// tinyAVR `CLKCTRL`. Not for external use.
pub trait MainClkControl {
    /// Writes `MCLKCTRLB`. `Some(div)` enables the prescaler at that division;
    /// `None` disables it. Protected, so the caller must unlock CCP just before.
    fn write_prescaler(&self, div: Option<ClkPrescaler>);
    /// Whether a clock switch is still in progress (`MCLKSTATUS.SOSC`).
    fn clock_switching(&self) -> bool;
}

/// Sets the tinyAVR main-clock prescaler and waits for the switch to settle.
/// `None` disables the prescaler, so `CLK_PER` becomes the base oscillator
/// frequency (see [`TinyBaseFreq`]).
///
/// `MCLKCTRLB` is written whole. The CCP unlock and the protected write happen
/// with interrupts masked so the unlock window cannot be interrupted.
///
/// # Panics
/// Panics if the clock switch does not complete within the defensive spin
/// budget, which means the peripheral is broken or misconfigured.
#[inline(always)]
pub fn set_main_clock_prescaler<C: CcpUnlock, K: MainClkControl>(
    cpu: &C,
    clkctrl: &K,
    div: Option<ClkPrescaler>,
) {
    avr_device::interrupt::free(|_| {
        cpu.unlock_ioreg();
        clkctrl.write_prescaler(div);
    });
    crate::wait::spin_until(|| !clkctrl.clock_switching());
}

macro_rules! impl_main_clk_control {
    ($CLKCTRL:ty) => {
        impl MainClkControl for $CLKCTRL {
            #[inline(always)]
            fn write_prescaler(&self, div: Option<ClkPrescaler>) {
                self.mclkctrlb().write(|w| match div {
                    None => w.pen().clear_bit(),
                    Some(d) => {
                        match d {
                            ClkPrescaler::Div2 => w.pdiv()._2x(),
                            ClkPrescaler::Div4 => w.pdiv()._4x(),
                            ClkPrescaler::Div6 => w.pdiv()._6x(),
                            ClkPrescaler::Div8 => w.pdiv()._8x(),
                            ClkPrescaler::Div10 => w.pdiv()._10x(),
                            ClkPrescaler::Div12 => w.pdiv()._12x(),
                            ClkPrescaler::Div16 => w.pdiv()._16x(),
                            ClkPrescaler::Div24 => w.pdiv()._24x(),
                            ClkPrescaler::Div32 => w.pdiv()._32x(),
                            ClkPrescaler::Div48 => w.pdiv()._48x(),
                            ClkPrescaler::Div64 => w.pdiv()._64x(),
                        };
                        w.pen().set_bit()
                    }
                });
            }
            fn clock_switching(&self) -> bool {
                self.mclkstatus().read().sosc().bit_is_set()
            }
        }
    };
}

#[cfg(feature = "attiny406")]
impl_main_clk_control!(avr_device::attiny406::CLKCTRL);
#[cfg(feature = "attiny416")]
impl_main_clk_control!(avr_device::attiny416::CLKCTRL);

#[cfg(feature = "attiny406")]
impl_ccp_unlock!(avr_device::attiny406::CPU);
#[cfg(feature = "attiny416")]
impl_ccp_unlock!(avr_device::attiny416::CPU);
