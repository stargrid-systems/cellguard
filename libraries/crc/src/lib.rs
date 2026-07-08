//! Small, table-free CRC implementations for corruption detection.
//!
//! Two variants are provided:
//!
//! - [`Crc32`]: reflected CRC-32 (IEEE 802.3), polynomial `0xEDB8_8320`. Used
//!   for firmware image integrity.
//! - [`Crc16`]: CRC-16/MODBUS, polynomial `0xA001`, initial value `0xFFFF`, no
//!   final xor. Used for the communication frame header.
//!
//! Both are bitwise and table-free to keep code size small on the target, and
//! both stream: feeding data in chunks yields the same result as feeding it all
//! at once. These detect corruption only, not tampering.
#![no_std]
#![warn(missing_docs)]

/// Incremental CRC-32 (IEEE 802.3) calculator.
#[derive(Debug, Clone)]
pub struct Crc32 {
    state: u32,
}

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

impl Crc32 {
    /// Creates a calculator in its initial state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: 0xFFFF_FFFF,
        }
    }

    /// Feeds `data` into the calculator.
    pub fn update(&mut self, data: &[u8]) {
        for &byte in data {
            self.state ^= u32::from(byte);
            for _ in 0..8 {
                // Branchless conditional xor: `mask` is all-ones when the low
                // bit is set and all-zeros otherwise.
                let mask = (self.state & 1).wrapping_neg();
                self.state = (self.state >> 1) ^ (0xEDB8_8320 & mask);
            }
        }
    }

    /// Consumes the calculator and returns the final CRC-32 value.
    #[must_use]
    pub const fn finalize(self) -> u32 {
        self.state ^ 0xFFFF_FFFF
    }
}

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

/// Computes the CRC-32 of `data` in one call.
#[must_use]
pub fn checksum32(data: &[u8]) -> u32 {
    let mut crc = Crc32::new();
    crc.update(data);
    crc.finalize()
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
    use super::{Crc16, Crc32, checksum16, checksum32};

    #[test]
    fn crc32_known_vectors() {
        assert_eq!(checksum32(b""), 0x0000_0000);
        assert_eq!(checksum32(b"a"), 0xE8B7_BE43);
        assert_eq!(checksum32(b"123456789"), 0xCBF4_3926);
        assert_eq!(checksum32(b"The quick brown fox jumps over the lazy dog"), 0x414F_A339);
    }

    #[test]
    fn crc16_modbus_known_vectors() {
        // CRC-16/MODBUS check value.
        assert_eq!(checksum16(b"123456789"), 0x4B37);
    }

    #[test]
    fn chunked_matches_whole() {
        let data = b"The quick brown fox jumps over the lazy dog";
        let mut c32 = Crc32::new();
        c32.update(&data[..10]);
        c32.update(&data[10..]);
        assert_eq!(c32.finalize(), checksum32(data));

        let mut c16 = Crc16::new();
        c16.update(&data[..10]);
        c16.update(&data[10..]);
        assert_eq!(c16.finalize(), checksum16(data));
    }
}
