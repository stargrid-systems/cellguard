//! A [`UpdiLink`] over the ATtiny406 one-wire USART.
//!
//! The PROG tiny has a single USART, shared with the cellcore UART link
//! through the U1004 mux and lent here only for the duration of a flash. In
//! UPDI mode its TxD and RxD are coupled onto the target's UPDI line, so every
//! byte sent echoes back. [`UsartUpdiLink::send_byte`] consumes that echo.

use avrxt_hal::usart::{Usart, UsartInstance};
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
    /// All transport failures map to one session status.
    type Error = ();

    fn break_(&mut self) -> Result<(), ()> {
        self.usart.send_break();
        // The break echoes back. Drop it, ignoring a timeout since BREAK is
        // best-effort.
        let _ = self.usart.read_byte();
        Ok(())
    }

    fn send_byte(&mut self, byte: u8) -> Result<(), ()> {
        self.usart.write_byte(byte);
        // One-wire: the byte just sent is echoed. Consume it.
        match self.usart.read_byte() {
            Ok(_) => Ok(()),
            Err(_) => Err(()),
        }
    }

    fn recv_byte(&mut self) -> Result<u8, ()> {
        self.usart.read_byte().map_err(|_| ())
    }
}
