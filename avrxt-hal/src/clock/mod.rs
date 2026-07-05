//! Main clock configuration (CLKCTRL).
//!
//! Configuration-change-protected registers are opened with [`CcpUnlock`]. The
//! per-family clock control lives in submodules: AVR128 selects the internal
//! high-frequency oscillator ([`set_oschf`]), tinyAVR runs from `OSC20M` and
//! adjusts the main-clock prescaler ([`set_main_clock_prescaler`]). Both do the
//! CCP unlock plus the protected write inside [`avr_device::interrupt::free`], so
//! an interrupt cannot land in the unlock window.

#[cfg(feature = "_avr128")]
mod avr128;
#[cfg(feature = "_tinyavr")]
mod tiny;

#[cfg(feature = "_avr128")]
pub use self::avr128::{HfFreq, OscControl, set_oschf};
#[cfg(feature = "_tinyavr")]
pub use self::tiny::{ClkPrescaler, MainClkControl, TinyBaseFreq, set_main_clock_prescaler};

/// Unlocks configuration-change-protected registers. Implemented for each
/// device's `CPU`. Not for external use.
pub trait CcpUnlock {
    /// Writes the IOREG signature to `CPU.CCP`, opening the ~4-cycle window in
    /// which the next protected store is accepted.
    fn unlock_ioreg(&self);
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

#[cfg(feature = "avr128db48")]
impl_ccp_unlock!(avr_device::avr128db48::CPU);
#[cfg(feature = "avr128db64")]
impl_ccp_unlock!(avr_device::avr128db64::CPU);
#[cfg(feature = "avr128da64")]
impl_ccp_unlock!(avr_device::avr128da64::CPU);
#[cfg(feature = "attiny406")]
impl_ccp_unlock!(avr_device::attiny406::CPU);
#[cfg(feature = "attiny416")]
impl_ccp_unlock!(avr_device::attiny416::CPU);
