//! CRC-16/MODBUS, polynomial `0xA001`, initial value `0xFFFF`, no final xor.

/// Incremental CRC-16/MODBUS calculator.
#[derive(Debug, Clone)]
pub struct Crc16 {
    state: u16,
}

impl Default for Crc16 {
    fn default() -> Self {
        Self::new()
    }
}

impl Crc16 {
    /// Creates a calculator in its initial state.
    #[must_use]
    pub const fn new() -> Self {
        Self { state: 0xFFFF }
    }

    /// Feeds `data` into the calculator.
    pub fn update(&mut self, data: &[u8]) {
        for &byte in data {
            self.state ^= u16::from(byte);
            for _ in 0..8 {
                let mask = (self.state & 1).wrapping_neg();
                self.state = (self.state >> 1) ^ (0xA001 & mask);
            }
        }
    }

    /// Consumes the calculator and returns the final CRC-16 value.
    #[must_use]
    pub const fn finalize(self) -> u16 {
        self.state
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
    fn chunked_matches_whole() {
        let data = b"The quick brown fox jumps over the lazy dog";
        let mut crc = Crc16::new();
        crc.update(data.get(..10).unwrap());
        crc.update(data.get(10..).unwrap());
        assert_eq!(crc.finalize(), checksum16(data));
    }
}
