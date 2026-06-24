use crate::Model;

/// Largest command header: opcode plus up to three address bytes.
pub const HEADER_MAX: usize = 4;

/// Returns true if a `len`-byte access starting at `address` stays within
/// `limit` bytes.
///
/// All arithmetic is done in `u32` with overflow treated as out of bounds, so
/// the check is correct on targets with a 16-bit `usize` (AVR).
pub fn range_in_bounds(address: u32, len: usize, limit: u32) -> bool {
    u32::try_from(len).is_ok_and(|len| address.checked_add(len).is_some_and(|end| end <= limit))
}

/// Encodes `opcode` followed by the model's address bytes into `buf`.
///
/// Returns the populated prefix of `buf`.
#[expect(
    clippy::indexing_slicing,
    reason = "buf is HEADER_MAX bytes and holds the opcode plus up to three address bytes"
)]
pub fn encode_header(
    model: Model,
    buf: &mut [u8; HEADER_MAX],
    opcode: u8,
    address: u32,
) -> &mut [u8] {
    buf[0] = opcode;
    let n = model.encode_address(&mut buf[1..], address);
    &mut buf[..=n]
}

/// Splits a write into chunks that each stay within a single page.
///
/// The CAT25 family wraps within a page instead of advancing past its end, so a
/// write that crosses a page boundary must be issued as separate page writes.
/// Each item is the device address of the chunk and its length in bytes.
pub struct PageChunks {
    address: u32,
    remaining: usize,
    page: u32,
}

impl PageChunks {
    pub fn new(address: u32, len: usize, page_size: u16) -> Self {
        Self {
            address,
            remaining: len,
            page: u32::from(page_size),
        }
    }
}

impl Iterator for PageChunks {
    type Item = (u32, usize);

    #[expect(
        clippy::cast_possible_truncation,
        reason = "a page is at most 256 bytes, so a chunk fits in both usize and u32"
    )]
    fn next(&mut self) -> Option<Self::Item> {
        if self.remaining == 0 {
            return None;
        }
        let space = (self.page - self.address % self.page) as usize;
        let chunk = space.min(self.remaining);
        let address = self.address;
        self.address += chunk as u32;
        self.remaining -= chunk;
        Some((address, chunk))
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec::Vec;

    use super::*;
    use crate::CAT25128;

    #[test]
    fn range_in_bounds_checks() {
        assert!(range_in_bounds(0, 16_384, 16_384));
        assert!(range_in_bounds(16_383, 1, 16_384));
        assert!(range_in_bounds(0, 0, 16_384));
        assert!(!range_in_bounds(16_383, 2, 16_384));
        assert!(!range_in_bounds(16_384, 1, 16_384));
        // Address plus length must not wrap.
        assert!(!range_in_bounds(u32::MAX, 1, 16_384));
    }

    #[test]
    fn encode_header_writes_opcode_and_address() {
        let mut buf = [0u8; HEADER_MAX];
        let header = encode_header(CAT25128, &mut buf, 0x03, 0x1234);
        assert_eq!(header, &[0x03, 0x12, 0x34]);
    }

    #[test]
    fn page_chunks_single_page() {
        let chunks: Vec<_> = PageChunks::new(0, 10, 64).collect();
        assert_eq!(chunks, [(0, 10)]);
    }

    #[test]
    fn page_chunks_splits_on_boundary() {
        // Page size 64. Starting at 63 with 3 bytes spans two pages.
        let chunks: Vec<_> = PageChunks::new(63, 3, 64).collect();
        assert_eq!(chunks, [(63, 1), (64, 2)]);
    }

    #[test]
    fn page_chunks_multiple_full_pages() {
        let chunks: Vec<_> = PageChunks::new(0, 130, 64).collect();
        assert_eq!(chunks, [(0, 64), (64, 64), (128, 2)]);
    }

    #[test]
    fn page_chunks_empty() {
        assert!(PageChunks::new(0, 0, 64).next().is_none());
    }
}
