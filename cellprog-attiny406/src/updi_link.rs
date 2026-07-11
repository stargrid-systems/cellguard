//! A [`UpdiLink`] over the ATtiny406 one-wire USART.
//!
//! The PROG tiny's USART TxD and RxD are joined externally onto the target UPDI
//! wire (through the mux and series resistors), so every transmitted byte is
//! echoed back. [`UsartUpdiLink::send`] consumes that echo, so the `updi` stack
//! sees a clean request/response link.

use avrxt_hal::usart::{Error, Usart, UsartInstance};
use updi::UpdiLink;

/// A UPDI transport backed by a HAL USART in one-wire 8E2 mode.
pub struct UsartUpdiLink<T: UsartInstance> {
    usart: Usart<T>,
}

impl<T: UsartInstance> UsartUpdiLink<T> {
    /// Wraps a USART already configured for UPDI (`Frame::EIGHT_E_2`).
    pub const fn new(usart: Usart<T>) -> Self {
        Self { usart }
    }
}

impl<T: UsartInstance> UpdiLink for UsartUpdiLink<T> {
    type Error = Error;

    fn break_(&mut self) -> Result<(), Error> {
        self.usart.send_break();
        // The break byte echoes back on the shared line. Drop it, ignoring a
        // timeout since a BREAK is best-effort.
        let _ = self.usart.read_byte();
        Ok(())
    }

    fn send(&mut self, data: &[u8]) -> Result<(), Error> {
        for &byte in data {
            self.usart.write_byte(byte);
            // One-wire: the byte just sent is echoed. Consume it.
            self.usart.read_byte()?;
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8]) -> Result<(), Error> {
        for byte in buf.iter_mut() {
            *byte = self.usart.read_byte()?;
        }
        Ok(())
    }
}
