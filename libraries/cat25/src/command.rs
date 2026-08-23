/// Enable Write Operations
pub const WREN: u8 = 0b0000_0110;
/// Disable Write Operations
pub const WRDI: u8 = 0b0000_0100;
/// Read Status Register
pub const RDSR: u8 = 0b0000_0101;
/// Write Status Register
pub const WRSR: u8 = 0b0000_0001;
/// Read Data from Memory
pub const READ: u8 = 0b0000_0011;
/// Write Data to Memory
pub const WRITE: u8 = 0b0000_0010;

/// Largest command header: opcode plus up to three address bytes.
pub const HEADER_MAX: usize = 4;

/// Returns true if a `len`-byte access starting at `address` stays within
/// `limit` bytes.
pub fn range_in_bounds(address: u32, len: usize, limit: u32) -> bool {
    u32::try_from(len).is_ok_and(|len| address.checked_add(len).is_some_and(|end| end <= limit))
}

/// Splits a write into chunks that each stay within a single page.
///
/// `page_size` must be a power of two. The [`crate::Model`] constants enforce
/// this at compile time.
///
/// The CAT25 family wraps within a page instead of advancing past its end, so a
/// write that crosses a page boundary must be issued as separate page writes.
/// Each item is the device address of the chunk and its length in bytes.
pub struct PageChunks {
    start: u32,
    end: u32,
    page_size: u32,
}

impl PageChunks {
    pub fn new(start: u32, len: usize, page_size: u16) -> Self {
        Self {
            start,
            end: start.saturating_add(u32::try_from(len).unwrap_or(u32::MAX)),
            page_size: u32::from(page_size),
        }
    }
}

impl Iterator for PageChunks {
    type Item = (u32, usize);

    fn next(&mut self) -> Option<Self::Item> {
        if self.start >= self.end {
            return None;
        }
        let address = self.start;
        // Page sizes are powers of two (see `Model`), so a mask replaces the
        // modulo. AVR has no divide instruction: the modulo would link the
        // 1 KiB software divider.
        let left_in_page = self.page_size - (self.start & (self.page_size - 1));
        let chunk_end = self.end.min(self.start.saturating_add(left_in_page));
        self.start = chunk_end;
        // The chunk spans at most one page, so its length fits in a usize.
        Some((
            address,
            usize::try_from(chunk_end - address).unwrap_or(usize::MAX),
        ))
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
        let header = CAT25128.encode_header(&mut buf, 0x03, 0x1234);
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
