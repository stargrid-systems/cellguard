//! CRC-32 (IEEE 802.3) for image integrity checks.
//!
//! This is the reflected CRC-32 with polynomial `0xEDB8_8320`, the same variant
//! used by zlib and gzip. The implementation is table-free to keep the code
//! small on the target. It is used for corruption detection, not authenticity.

/// Incremental CRC-32 (IEEE 802.3) calculator.
///
/// Feed data with [`Crc32::update`] and read the result with
/// [`Crc32::finalize`]. Feeding in chunks yields the same result as feeding the
/// whole input at once, so the payload can be streamed from external storage.
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

/// Computes the CRC-32 of `data` in one call.
#[must_use]
pub fn checksum(data: &[u8]) -> u32 {
    let mut crc = Crc32::new();
    crc.update(data);
    crc.finalize()
}

#[cfg(test)]
mod tests {
    use super::checksum;

    #[test]
    fn known_vectors() {
        assert_eq!(checksum(b""), 0x0000_0000);
        assert_eq!(checksum(b"a"), 0xE8B7_BE43);
        assert_eq!(checksum(b"123456789"), 0xCBF4_3926);
        assert_eq!(checksum(b"The quick brown fox jumps over the lazy dog"), 0x414F_A339);
    }

    #[test]
    fn chunked_matches_whole() {
        let data = b"The quick brown fox jumps over the lazy dog";
        let whole = checksum(data);
        let mut crc = super::Crc32::new();
        crc.update(&data[..10]);
        crc.update(&data[10..]);
        assert_eq!(crc.finalize(), whole);
    }
}
