//! One-ramp PWM on the 12-bit timer TCD0.
//!
//! [`TcdPwm`] is generic over a [`TcdInstance`]. One-ramp mode generates a
//! single-slope waveform on compare channel A: the cycle is `top + 1`
//! counter ticks (`CMPBCLR` = `top`), and the output is high for the
//! `ticks` counter values ending at the ramp end. Duty updates are
//! double-buffered and applied at the end of the current cycle
//! (`CTRLE.SYNCEOC`), so the output never glitches. The timer must be in
//! its reset state (disabled). `PORTMUX` routing and pin direction for the
//! WO output are the application's responsibility.
//!
//! `FAULTCTRL`, which enables the output, is configuration-change
//! protected. [`TcdPwm::new`] takes the device's `CPU` (see
//! [`crate::clock::CcpUnlock`]) and does the unlock with interrupts masked.

use crate::clock::CcpUnlock;
use crate::wait::spin_until;

/// Largest usable `top`. The zero-duty encoding parks `CMPASET` above the
/// ramp end, so one compare value must stay above `top`.
pub const MAX_TOP: u16 = 0x0FFE;

/// `CMPASET` value for a constant-low output. Larger than any legal `top`,
/// so the set event never fires and the ramp-end clear keeps the output
/// low.
const ZERO_DUTY_CMP: u16 = 0x0FFF;

/// TCD0 counter-clock prescaler: `CLK_PER` divided by `SYNCPRES` and
/// `CNTPRES` combined.
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
}

/// Physical output pin for the generated waveform.
#[derive(Clone, Copy)]
pub enum Output {
    /// WOA.
    Woa,
    /// WOD. Mirrors the WOA waveform through the default `CMPDSEL`
    /// routing.
    Wod,
}

/// A TCD timer usable for one-ramp PWM. Implemented for each device's
/// `TCD0`. Not for external use.
pub trait TcdInstance {
    /// Configures one-ramp mode at zero duty and starts the timer from
    /// `CLK_PER` with the given prescaler.
    fn configure(&self, top: u16, prescaler: Prescaler);
    /// Enables the output in the CCP-protected `FAULTCTRL`. The caller must
    /// unlock `CPU.CCP` immediately before.
    fn enable_output(&self, output: Output);
    /// Buffers the compare pair for `ticks` on-time and strobes `SYNCEOC`.
    fn set_on_ticks(&self, top: u16, ticks: u16);
}

/// One-ramp PWM generator on a TCD timer.
pub struct TcdPwm<T: TcdInstance> {
    instance: T,
    top: u16,
}

impl<T: TcdInstance> TcdPwm<T> {
    /// Configures and starts one-ramp PWM at zero duty (constant low).
    ///
    /// # Panics
    ///
    /// Panics if `top` exceeds [`MAX_TOP`]: the zero-duty encoding needs a
    /// compare value above the ramp end.
    #[must_use]
    pub fn new<C: CcpUnlock>(
        cpu: &C,
        instance: T,
        top: u16,
        prescaler: Prescaler,
        output: Output,
    ) -> Self {
        assert!(top <= MAX_TOP);
        // FAULTCTRL first, while the timer is still disabled.
        avr_device::interrupt::free(|_| {
            cpu.unlock_ioreg();
            instance.enable_output(output);
        });
        instance.configure(top, prescaler);
        Self { instance, top }
    }

    /// Sets the on-time in counter ticks. Zero is a constant low output,
    /// `top` is high for the whole ramp except the wrap tick. The new value
    /// takes effect at the end of the current cycle.
    ///
    /// # Panics
    ///
    /// Panics if `ticks` exceeds the configured `top`.
    pub fn set_on_ticks(&mut self, ticks: u16) {
        assert!(ticks <= self.top);
        self.instance.set_on_ticks(self.top, ticks);
    }

    /// Releases the underlying peripheral.
    pub fn free(self) -> T {
        self.instance
    }
}

macro_rules! impl_tcd_instance {
    ($TCD:ty) => {
        impl TcdInstance for $TCD {
            fn configure(&self, top: u16, prescaler: Prescaler) {
                self.ctrlb().write(|w| w.wgmode().oneramp());
                self.cmpaset().write(|w| w.cmpaset().set(ZERO_DUTY_CMP));
                self.cmpaclr().write(|w| w.cmpaclr().set(top));
                self.cmpbclr().write(|w| w.compbclr().set(top));
                spin_until(|| self.status().read().enrdy().bit_is_set());
                self.ctrla().write(|w| {
                    let w = w.clksel().clkper();
                    let w = match prescaler {
                        Prescaler::Div1 => w.syncpres().div1().cntpres().div1(),
                        Prescaler::Div2 => w.syncpres().div2().cntpres().div1(),
                        Prescaler::Div4 => w.syncpres().div4().cntpres().div1(),
                        Prescaler::Div8 => w.syncpres().div8().cntpres().div1(),
                        Prescaler::Div16 => w.syncpres().div4().cntpres().div4(),
                        Prescaler::Div32 => w.syncpres().div8().cntpres().div4(),
                        Prescaler::Div64 => w.syncpres().div2().cntpres().div32(),
                        Prescaler::Div128 => w.syncpres().div4().cntpres().div32(),
                        Prescaler::Div256 => w.syncpres().div8().cntpres().div32(),
                    };
                    w.enable().set_bit()
                });
            }
            fn enable_output(&self, output: Output) {
                self.faultctrl().write(|w| match output {
                    Output::Woa => w.cmpaen().set_bit(),
                    Output::Wod => w.cmpden().set_bit(),
                });
            }
            fn set_on_ticks(&self, top: u16, ticks: u16) {
                spin_until(|| self.status().read().cmdrdy().bit_is_set());
                let set = if ticks == 0 {
                    ZERO_DUTY_CMP
                } else {
                    top - ticks
                };
                self.cmpaset().write(|w| w.cmpaset().set(set));
                self.cmpaclr().write(|w| w.cmpaclr().set(top));
                self.ctrle().write(|w| w.synceoc().set_bit());
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_tcd_instance!(avr_device::avr128db48::TCD0);
#[cfg(feature = "avr128db64")]
impl_tcd_instance!(avr_device::avr128db64::TCD0);
#[cfg(feature = "avr128da64")]
impl_tcd_instance!(avr_device::avr128da64::TCD0);
