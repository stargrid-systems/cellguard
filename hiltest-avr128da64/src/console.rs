//! USART5 console line I/O.
//!
//! PORTMUX routing (ALT1, PG4/PG5) and pin directions are set in `main`.
//! The console can rebuild its baud divisor after a main-clock switch.

use avr_device::avr128da64 as pac;
use avrxt_hal::usart::{BaudUnattainable, Error, Frame, Usart};
use ufmt::uWrite;

/// Receive timeout per poll attempt, in nominal ms. `budget_ms` undercounts
/// the real loop cost by 5-12x, so one attempt really lasts up to a few
/// hundred ms.
const RX_TIMEOUT_MS: u32 = 20;

/// The debug console on USART5.
pub struct Console {
    usart: Option<Usart<pac::USART5>>,
    baud: u32,
}

impl Console {
    /// Brings up USART5 as 8N1 at `baud`.
    ///
    /// # Panics
    /// Panics when `baud` is unattainable for `f_cpu_hz`. At boot this loops
    /// through reset silently, but the boot values are compile-time constants
    /// that are known to be attainable.
    pub fn new(usart5: pac::USART5, f_cpu_hz: u32, baud: u32) -> Self {
        Self {
            usart: Some(build(usart5, f_cpu_hz, baud)),
            baud,
        }
    }

    fn usart(&mut self) -> &mut Usart<pac::USART5> {
        // The slot is only empty inside `set_f_cpu`, which refills it before
        // returning.
        self.usart.as_mut().unwrap_or_else(|| crate::halt())
    }

    /// Recomputes the baud divisor for a new main-clock frequency. Call with
    /// the transmitter drained (see [`Self::flush`]).
    ///
    /// # Panics
    /// Panics when the configured baud is unattainable for `f_cpu_hz`. The
    /// panic is reported through the resume record like any test panic.
    pub fn set_f_cpu(&mut self, f_cpu_hz: u32) {
        let Some(usart) = self.usart.take() else {
            crate::halt()
        };
        self.usart = Some(build(usart.free(), f_cpu_hz, self.baud));
    }

    /// Drains the transmit shift register.
    pub fn flush(&mut self) {
        let _ = embedded_io::Write::flush(self.usart());
    }

    /// Reads one LF-terminated line into `buf`, without the terminator.
    ///
    /// CR bytes are dropped. Bytes past the end of `buf` are discarded, so an
    /// overlong line comes back truncated. Returns [`None`] once `attempts`
    /// receive timeouts pass without the newline arriving.
    pub fn read_line(&mut self, buf: &mut [u8], attempts: u32) -> Option<usize> {
        let mut len = 0;
        let mut timeouts = 0;
        loop {
            match self.usart().read_byte() {
                Ok(b'\n') => return Some(len),
                Ok(b'\r') => {}
                Ok(byte) => {
                    if let Some(slot) = buf.get_mut(len) {
                        *slot = byte;
                        len += 1;
                    }
                }
                Err(Error::Timeout) => {
                    timeouts += 1;
                    if timeouts >= attempts {
                        return None;
                    }
                }
            }
        }
    }

    /// Writes one byte as two uppercase hex digits.
    pub fn write_hex_byte(&mut self, byte: u8) {
        let Ok(()) = self.write_char(hex_char(byte >> 4));
        let Ok(()) = self.write_char(hex_char(byte));
    }
}

impl uWrite for Console {
    type Error = core::convert::Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        self.usart().write_str(s)
    }
}

/// Builds the USART, panicking on an unattainable baud so the failure is
/// loud.
fn build(usart5: pac::USART5, f_cpu_hz: u32, baud: u32) -> Usart<pac::USART5> {
    Usart::builder(usart5, f_cpu_hz)
        .baud(baud)
        .rx_timeout_ms(RX_TIMEOUT_MS)
        .frame(Frame::EIGHT_N_1)
        .build()
        .unwrap_or_else(|BaudUnattainable| panic!("console baud unattainable"))
}

pub const fn hex_char(nibble: u8) -> char {
    let n = nibble & 0xF;
    let byte = if n < 10 { b'0' + n } else { b'A' + (n - 10) };
    byte as char
}
