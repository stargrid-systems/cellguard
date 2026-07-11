//! A typestate builder for [`Usart`].
//!
//! [`Usart::builder`](super::Usart::builder) returns a [`Builder`] that must be
//! given a baud rate and a [`Frame`] before [`Builder::build`] is callable. The
//! two are required at compile time, so nothing is assumed for the caller. The
//! receive timeout keeps a default and is the only optional knob.

use super::{
    BaudUnattainable, DEFAULT_RX_TIMEOUT_MS, Frame, Usart, UsartInstance, baud_reg_checked,
};

/// A builder field that has not been set yet. `build` is not implemented while
/// either the baud or the frame still has this type.
pub struct Unset;

/// Builds a [`Usart`]. The `B` and `F` type parameters track whether the baud
/// and frame have been set. They start as [`Unset`] and become `u32` and
/// [`Frame`] once set, which is when [`Builder::build`] becomes callable.
pub struct Builder<T, B, F> {
    instance: T,
    f_cpu_hz: u32,
    baud: B,
    frame: F,
    rx_timeout_ms: u32,
}

impl<T: UsartInstance> Builder<T, Unset, Unset> {
    pub(super) fn new(instance: T, f_cpu_hz: u32) -> Self {
        Self {
            instance,
            f_cpu_hz,
            baud: Unset,
            frame: Unset,
            rx_timeout_ms: DEFAULT_RX_TIMEOUT_MS,
        }
    }
}

impl<T, B, F> Builder<T, B, F> {
    /// Sets the baud rate in bits/s.
    #[must_use]
    pub fn baud(self, baud: u32) -> Builder<T, u32, F> {
        Builder {
            instance: self.instance,
            f_cpu_hz: self.f_cpu_hz,
            baud,
            frame: self.frame,
            rx_timeout_ms: self.rx_timeout_ms,
        }
    }

    /// Sets the frame format.
    #[must_use]
    pub fn frame(self, frame: Frame) -> Builder<T, B, Frame> {
        Builder {
            instance: self.instance,
            f_cpu_hz: self.f_cpu_hz,
            baud: self.baud,
            frame,
            rx_timeout_ms: self.rx_timeout_ms,
        }
    }

    /// Overrides the receive timeout in milliseconds (approximate, derived from
    /// `f_cpu_hz`). Defaults to one second.
    #[must_use]
    pub fn rx_timeout_ms(mut self, rx_timeout_ms: u32) -> Self {
        self.rx_timeout_ms = rx_timeout_ms;
        self
    }
}

impl<T: UsartInstance> Builder<T, u32, Frame> {
    /// Configures the peripheral and returns the [`Usart`].
    ///
    /// # Errors
    ///
    /// Returns [`BaudUnattainable`] when the baud rate cannot be represented
    /// for `f_cpu_hz`.
    pub fn build(self) -> Result<Usart<T>, BaudUnattainable> {
        let reg = baud_reg_checked(self.f_cpu_hz, self.baud)?;
        self.instance.configure(reg, self.frame);
        Ok(Usart {
            instance: self.instance,
            f_cpu_hz: self.f_cpu_hz,
            baud_reg: reg,
            rx_budget: crate::wait::budget_ms(self.f_cpu_hz, self.rx_timeout_ms),
            tx_pending: false,
        })
    }
}
