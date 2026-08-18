//! The UPDI transport seam.
//!
//! [`UpdiLink`] is the single hardware boundary of this crate. A concrete
//! target implements it over a USART in one-wire mode. Because the line is
//! shared, every byte the host sends is echoed back. The byte-wise methods
//! consume that echo, so [`UpdiLink::recv_byte`] returns only target bytes.
//! The instruction set that runs over this seam lives in
//! [`driver`](crate::driver).

/// The half-duplex, single-wire transport to the target's UPDI slave.
///
/// Implementors provide the byte-wise primitives. The slice methods have
/// default impls on top. Instructions are two to four bytes, and building
/// them as stack arrays just to pass a slice costs more flash on small
/// targets than sending the bytes directly.
pub trait UpdiLink {
    /// Transport error.
    type Error;

    /// Sends a BREAK to reset the target's UPDI state machine.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport fails.
    fn break_(&mut self) -> Result<(), Self::Error>;

    /// Sends one byte, consuming its echo so it does not appear on
    /// [`UpdiLink::recv_byte`].
    ///
    /// # Errors
    ///
    /// Returns an error if the transport fails.
    fn send_byte(&mut self, byte: u8) -> Result<(), Self::Error>;

    /// Receives one byte from the target.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport fails or times out.
    fn recv_byte(&mut self) -> Result<u8, Self::Error>;

    /// Sends `data`, consuming each byte's echo.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport fails.
    fn send(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        for &byte in data {
            self.send_byte(byte)?;
        }
        Ok(())
    }

    /// Receives exactly `buf.len()` bytes from the target.
    ///
    /// # Errors
    ///
    /// Returns an error if the transport fails or times out.
    fn recv(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        for byte in buf.iter_mut() {
            *byte = self.recv_byte()?;
        }
        Ok(())
    }
}
