//! CRC-16/MODBUS, polynomial `0xA001`, initial value `0xFFFF`, no final xor.

use crate::bitwise::CrcCore;

/// Incremental CRC-16/MODBUS calculator.
#[derive(Debug, Clone)]
pub struct Crc16(CrcCore<u16>);

impl Default for Crc16 {
    fn default() -> Self {
        Self::new()
    }
}

impl Crc16 {
    /// Creates a calculator in its initial state.
    #[must_use]
    pub const fn new() -> Self {
        Self(CrcCore::new(0xFFFF))
    }

    /// Feeds `data` into the calculator.
    pub fn update(&mut self, data: &[u8]) {
        self.0.update(data);
    }

    /// Consumes the calculator and returns the final CRC-16 value.
    #[must_use]
    pub const fn finalize(self) -> u16 {
        // CRC-16/MODBUS has no final xor.
        self.0.state()
    }
}

/// Computes the CRC-16/MODBUS of `data` in one call.
#[must_use]
pub fn checksum16(data: &[u8]) -> u16 {
    let mut crc = Crc16::new();
    crc.update(data);
    crc.finalize()
}

#[cfg(test)]
mod tests {
    use super::{Crc16, checksum16};

    #[test]
    fn modbus_known_vector() {
        assert_eq!(checksum16(b"123456789"), 0x4B37);
    }

    #[test]
    fn empty_input() {
        assert_eq!(checksum16(b""), 0xFFFF);
    }

    #[test]
    fn chunked_matches_whole() {
        let data = b"The quick brown fox jumps over the lazy dog";
        let mut crc = Crc16::new();
        crc.update(data.get(..10).unwrap());
        crc.update(data.get(10..).unwrap());
        assert_eq!(crc.finalize(), checksum16(data));
    }
}
