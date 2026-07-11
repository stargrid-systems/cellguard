//! The UPDI transport seam.
//!
//! [`UpdiLink`] is the single hardware boundary of this crate. A concrete
//! target implements it over a USART in one-wire mode. Because the line is
//! shared, every byte the host sends is echoed back. [`UpdiLink::send`] must
//! consume that echo, so [`UpdiLink::recv`] returns only target bytes. The
//! instruction set that runs over this seam lives in [`driver`](crate::driver).

/// The half-duplex, single-wire transport to the target's UPDI slave.
pub trait UpdiLink {
    /// Transport error.
    type Error;

    /// Sends a BREAK to reset the target's UPDI state machine.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport fails.
    fn break_(&mut self) -> Result<(), Self::Error>;

    /// Sends `data`, consuming its echo so it does not appear on `recv`.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport fails.
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error>;

    /// Receives exactly `buf.len()` bytes from the target.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport fails or times out.
    fn recv(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;
}
