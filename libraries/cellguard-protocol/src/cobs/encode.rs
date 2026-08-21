//! Streaming COBS encoder.

/// The largest number of data bytes that fit in one COBS block.
const MAX_DATA_PER_BLOCK: usize = 0xFF - 1;

/// A streaming COBS encoder.
///
/// Built from the frame to send, it yields one wire byte per [`Encoder::pull`]
/// until it returns `None`. The final `0x00` delimiter is included.
///
/// The state is plain indices into the frame, so `pull` re-slices nothing and
/// the code stays small on flash-limited targets.
pub struct Encoder<'a> {
    data: &'a [u8],
    pos: usize,
    /// End of the current block's data, exclusive.
    end: usize,
    /// Start of the block after the current one. One past `end` when the
    /// current block was terminated by a zero byte.
    next: usize,
    /// The current block implies no trailing zero (it filled all 254 data
    /// bytes).
    partial: bool,
    /// The next pull emits the frame delimiter.
    terminate: bool,
    done: bool,
}

impl<'a> Encoder<'a> {
    /// Creates an encoder for `data`.
    #[must_use]
    pub const fn new(data: &'a [u8]) -> Self {
        Self {
            data,
            pos: 0,
            end: 0,
            next: 0,
            partial: false,
            terminate: false,
            done: false,
        }
    }

    /// Returns the next wire byte, or `None` once the frame is complete.
    #[must_use]
    pub fn pull(&mut self) -> Option<u8> {
        if self.pos < self.end {
            let byte = *self.data.get(self.pos)?;
            self.pos += 1;
            return Some(byte);
        }
        if self.terminate {
            self.terminate = false;
            self.done = true;
            return Some(0x00);
        }
        if self.done {
            return None;
        }
        // No data left. A partial block ends the frame outright, any other
        // needs an empty block (code 0x01) to emit its implied trailing zero.
        let rest = self.data.get(self.next..).unwrap_or(&[]);
        if rest.is_empty() {
            if self.partial {
                self.done = true;
                return Some(0x00);
            }
            self.terminate = true;
            return Some(0x01);
        }
        // Emit the code byte of the next block: it ends at a zero within the
        // next 254 bytes, else runs 254 bytes or to the end of the data.
        let scan_len = rest.len().min(MAX_DATA_PER_BLOCK);
        let scan = rest.get(..scan_len).unwrap_or(&[]);
        let idx = scan.iter().position(|&b| b == 0);
        let block_len = idx.unwrap_or(scan_len);
        self.pos = self.next;
        self.end = self.next + block_len;
        self.next = self.end + usize::from(idx.is_some());
        if idx.is_some() {
            self.partial = false;
        } else if scan_len == MAX_DATA_PER_BLOCK {
            // A full block (code 0xFF) implies no trailing zero.
            self.partial = true;
        } else {
            // Final short block: only the delimiter follows.
            self.terminate = true;
        }
        #[expect(
            clippy::cast_possible_truncation,
            reason = "a block holds at most 254 data bytes, so the code byte fits a u8"
        )]
        let code = (block_len + 1) as u8;
        Some(code)
    }
}
