//! A typestate builder for [`Usart`].
//!
//! [`Usart::builder`](super::Usart::builder) returns a [`Builder`] that must be
//! given a baud rate and a [`Frame`] before [`Builder::build`] is callable. The
//! two are required at compile time, so nothing is assumed for the caller. The
//! receive timeout keeps a default and is the only optional knob.

use super::{DEFAULT_RX_TIMEOUT_MS, Frame, Usart, UsartInstance, baud_reg};

/// A builder field that has not been set yet. `build` is not implemented while
/// either the baud or the frame still has this type.
pub struct Unset;

/// A build-time configuration error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BuildError {
    /// The requested baud rate cannot be represented for the given `f_cpu_hz`.
    BaudUnattainable,
}

impl core::fmt::Display for BuildError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            BuildError::BaudUnattainable => f.write_str("baud rate unattainable for this clock"),
        }
    }
}

impl core::error::Error for BuildError {}

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
    /// Returns [`BuildError::BaudUnattainable`] when the baud rate cannot be
    /// represented for `f_cpu_hz`.
    pub fn build(self) -> Result<Usart<T>, BuildError> {
        let reg = baud_reg(self.f_cpu_hz, self.baud).ok_or(BuildError::BaudUnattainable)?;
        self.instance.configure(reg, self.frame);
        Ok(Usart {
            instance: self.instance,
            f_cpu_hz: self.f_cpu_hz,
            baud_reg: reg,
            rx_budget: crate::wait::budget_ms(self.f_cpu_hz, self.rx_timeout_ms),
        })
    }
}
