//! A [`UpdiLink`] over the ATtiny406 one-wire USART.
//!
//! The PROG tiny has a single USART. In UPDI mode (U1004 mux channel 1/3) its
//! TxD and RxD are coupled onto the target's UPDI line, so every transmitted
//! byte echoes back. [`UsartUpdiLink::send`] consumes that echo, so the `updi`
//! stack sees a clean request/response link.
//!
//! The link borrows the [`Usart`] rather than owning it, because the same USART
//! is also the UART command link to the cellcore (mux channel 0). The firmware
//! keeps the USART, switches its frame and the mux channel, and lends it here
//! only for the duration of a flash.

use avrxt_hal::usart::{Error, Usart, UsartInstance};
use updi::UpdiLink;

/// A UPDI transport backed by a borrowed HAL USART in one-wire 8E2 mode.
pub struct UsartUpdiLink<'a, T: UsartInstance> {
    usart: &'a mut Usart<T>,
}

impl<'a, T: UsartInstance> UsartUpdiLink<'a, T> {
    /// Wraps a USART already configured for UPDI (`Frame::EIGHT_E_2`).
    pub const fn new(usart: &'a mut Usart<T>) -> Self {
        Self { usart }
    }
}

impl<T: UsartInstance> UpdiLink for UsartUpdiLink<'_, T> {
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
