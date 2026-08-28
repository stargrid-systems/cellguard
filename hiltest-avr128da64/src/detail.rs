//! Buffer a test formats its dynamic result detail into.

use core::str;

use ufmt::uWrite;

use crate::console::hex_char;

/// Capacity in bytes. Sized for the longest detail, a full `twi-scan`
/// acknowledge list.
const CAP: usize = 352;

/// Byte buffer with a `uWrite` sink for single-token result details. Writes
/// past the capacity are dropped, so an overlong detail comes out truncated.
pub struct DetailBuf {
    buf: [u8; CAP],
    len: usize,
}

impl DetailBuf {
    /// Creates an empty buffer.
    pub const fn new() -> Self {
        Self {
            buf: [0; CAP],
            len: 0,
        }
    }

    /// The formatted detail.
    pub fn as_str(&self) -> &str {
        // Only ASCII is ever written, so the bytes are always valid UTF-8.
        str::from_utf8(self.buf.get(..self.len).unwrap_or(&[])).unwrap_or("?")
    }

    /// Appends one byte as two uppercase hex digits.
    pub fn write_hex_byte(&mut self, byte: u8) {
        let Ok(()) = self.write_char(hex_char(byte >> 4));
        let Ok(()) = self.write_char(hex_char(byte));
    }
}

impl Default for DetailBuf {
    fn default() -> Self {
        Self::new()
    }
}

impl uWrite for DetailBuf {
    type Error = core::convert::Infallible;

    fn write_str(&mut self, s: &str) -> Result<(), Self::Error> {
        for &byte in s.as_bytes() {
            if let Some(slot) = self.buf.get_mut(self.len) {
                *slot = byte;
                self.len += 1;
            }
        }
        Ok(())
    }
}
