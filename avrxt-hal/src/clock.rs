//! Main clock configuration (CLKCTRL).
//!
//! The AVR128 DB family boots on the internal high-frequency oscillator (OSCHF)
//! at 4 MHz. [`set_oschf`] selects another OSCHF frequency. `OSCHFCTRLA` is
//! configuration-change protected. The CCP IOREG unlock and the protected write
//! run inside [`avr_device::interrupt::free`], so an interrupt cannot land in
//! the 4-cycle window and void the unlock.
//!
//! tinyAVR parts have no OSCHF. They boot on the internal `OSC20M` oscillator
//! (16 or 20 MHz, chosen by the `OSCCFG` fuse) with the main-clock prescaler
//! enabled at /6. Use [`set_main_clock_prescaler`] to change or disable the
//! prescaler. The base frequency is a fuse setting, so the HAL cannot change it
//! at runtime; pass the fuse value in via [`TinyBaseFreq`] to compute
//! `CLK_PER`.

/// Internal high-frequency oscillator (OSCHF) frequency options. AVR128 only.
#[cfg(feature = "_avr128")]
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

#[cfg(feature = "_avr128")]
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

/// Unlocks configuration-change-protected registers. Implemented for each
/// device's `CPU`. Not for external use.
pub trait CcpUnlock {
    /// Writes the IOREG signature to `CPU.CCP`, opening the ~4-cycle window in
    /// which the next protected store is accepted.
    fn unlock_ioreg(&self);
}

/// Controls the high-frequency oscillator. Implemented for each AVR128 device's
/// `CLKCTRL`. Not for external use.
#[cfg(feature = "_avr128")]
pub trait OscControl {
    /// Writes `OSCHFCTRLA.FRQSEL`. This register is protected, so the caller
    /// must have just called [`CcpUnlock::unlock_ioreg`].
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
#[cfg(feature = "_avr128")]
#[inline(always)]
pub fn set_oschf<C: CcpUnlock, K: OscControl>(cpu: &C, clkctrl: &K, freq: HfFreq) {
    avr_device::interrupt::free(|_| {
        cpu.unlock_ioreg();
        clkctrl.write_frqsel(freq);
    });
    crate::wait::spin_until(|| clkctrl.oschf_stable());
}

macro_rules! impl_ccp_unlock {
    ($CPU:ty) => {
        impl CcpUnlock for $CPU {
            #[inline(always)]
            fn unlock_ioreg(&self) {
                self.ccp().write(|w| w.ccp().ioreg());
            }
        }
    };
}

#[cfg(feature = "_avr128")]
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
impl_ccp_unlock!(avr_device::avr128db48::CPU);
#[cfg(feature = "avr128db48")]
impl_osc_control!(avr_device::avr128db48::CLKCTRL);
#[cfg(feature = "avr128db64")]
impl_ccp_unlock!(avr_device::avr128db64::CPU);
#[cfg(feature = "avr128da64")]
impl_ccp_unlock!(avr_device::avr128da64::CPU);
#[cfg(feature = "avr128db64")]
impl_osc_control!(avr_device::avr128db64::CLKCTRL);
#[cfg(feature = "avr128da64")]
impl_osc_control!(avr_device::avr128da64::CLKCTRL);

// --- tinyAVR main clock (OSC20M + prescaler) ---

/// tinyAVR internal oscillator base frequency, selected by the `OSCCFG` fuse.
/// Not runtime-changeable; used only to compute `CLK_PER`.
#[cfg(feature = "_tinyavr")]
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TinyBaseFreq {
    Mhz16,
    Mhz20,
}

#[cfg(feature = "_tinyavr")]
impl TinyBaseFreq {
    /// The base oscillator frequency in Hz.
    #[must_use]
    pub const fn hz(self) -> u32 {
        match self {
            Self::Mhz16 => 16_000_000,
            Self::Mhz20 => 20_000_000,
        }
    }

    /// The resulting `CLK_PER` in Hz for the given prescaler setting. `None`
    /// means the prescaler is disabled (`CLK_PER` == base frequency).
    #[must_use]
    pub const fn clk_per_hz(self, div: Option<ClkPrescaler>) -> u32 {
        match div {
            Some(d) => self.hz() / d.divisor(),
            None => self.hz(),
        }
    }
}

/// tinyAVR main-clock prescaler division (`MCLKCTRLB.PDIV`).
#[cfg(feature = "_tinyavr")]
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

#[cfg(feature = "_tinyavr")]
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
#[cfg(feature = "_tinyavr")]
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
#[cfg(feature = "_tinyavr")]
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

#[cfg(feature = "_tinyavr")]
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
impl_ccp_unlock!(avr_device::attiny406::CPU);
#[cfg(feature = "attiny406")]
impl_main_clk_control!(avr_device::attiny406::CLKCTRL);
#[cfg(feature = "attiny416")]
impl_ccp_unlock!(avr_device::attiny416::CPU);
#[cfg(feature = "attiny416")]
impl_main_clk_control!(avr_device::attiny416::CLKCTRL);
