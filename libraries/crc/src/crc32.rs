//! Reflected CRC-32 (IEEE 802.3), polynomial `0xEDB8_8320`.

use crate::bitwise::CrcCore;

/// Incremental CRC-32 (IEEE 802.3) calculator.
#[derive(Debug, Clone)]
pub struct Crc32(CrcCore<u32>);

impl Default for Crc32 {
    fn default() -> Self {
        Self::new()
    }
}

impl Crc32 {
    /// Creates a calculator in its initial state.
    #[must_use]
    pub const fn new() -> Self {
        Self(CrcCore::new(0xFFFF_FFFF))
    }

    /// Feeds `data` into the calculator.
    pub fn update(&mut self, data: &[u8]) {
        self.0.update(data);
    }

    /// Consumes the calculator and returns the final CRC-32 value.
    #[must_use]
    pub const fn finalize(self) -> u32 {
        self.0.state() ^ 0xFFFF_FFFF
    }
}

/// Computes the CRC-32 of `data` in one call.
#[must_use]
pub fn checksum32(data: &[u8]) -> u32 {
    let mut crc = Crc32::new();
    crc.update(data);
    crc.finalize()
}

#[cfg(test)]
mod tests {
    use super::{Crc32, checksum32};

    #[test]
    fn known_vectors() {
        assert_eq!(checksum32(b""), 0x0000_0000);
        assert_eq!(checksum32(b"a"), 0xE8B7_BE43);
        assert_eq!(checksum32(b"123456789"), 0xCBF4_3926);
        assert_eq!(
            checksum32(b"The quick brown fox jumps over the lazy dog"),
            0x414F_A339
        );
    }

    #[test]
    fn chunked_matches_whole() {
        let data = b"The quick brown fox jumps over the lazy dog";
        let mut crc = Crc32::new();
        crc.update(data.get(..10).unwrap());
        crc.update(data.get(10..).unwrap());
        assert_eq!(crc.finalize(), checksum32(data));
    }
}
