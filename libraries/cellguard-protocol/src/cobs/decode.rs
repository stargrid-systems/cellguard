//! Streaming COBS decoder.

/// A streaming COBS decoder.
///
/// This is a pure state machine: it owns no buffer. Feed wire bytes one at a
/// time with [`Decoder::feed`], passing the same output buffer each call. When
/// the terminating `0x00` arrives, `feed` returns the decoded frame length,
/// and the frame is in the first that many bytes of the output buffer.
///
/// Owning no buffer means a caller can hold a `Decoder` and its output buffer
/// as separate fields without a self-referential borrow. The state is plain
/// integers, so an embedded decoder stays zero-initialized (`.bss`).
#[derive(Debug, Clone)]
pub struct Decoder {
    pos: usize,
    /// Data bytes still expected in the current block.
    remaining: u8,
    active: bool,
    /// Whether the current block implies no trailing zero (started by `0xFF`).
    partial: bool,
}

impl Default for Decoder {
    fn default() -> Self {
        Self::new()
    }
}

impl Decoder {
    /// Creates a decoder.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            pos: 0,
            remaining: 0,
            active: false,
            partial: false,
        }
    }

    /// Feeds one wire byte, writing decoded output into `out`.
    ///
    /// Returns `Ok(Some(len))` when a complete frame has been decoded into the
    /// first `len` bytes of `out`, or `Ok(None)` while a frame is still in
    /// progress. The same `out` buffer must be passed across the calls that
    /// make up one frame.
    ///
    /// # Errors
    ///
    /// Returns [`DecodeError::BufferTooSmall`] if the frame does not fit `out`,
    /// or [`DecodeError::InvalidFrame`] on a malformed frame.
    pub fn feed(&mut self, byte: u8, out: &mut [u8]) -> Result<Option<usize>, DecodeError> {
        if !self.active {
            // A zero is an empty frame, anything else starts the first block.
            if byte == 0x00 {
                return Ok(Some(0));
            }
            self.pos = 0;
            self.start_block(byte);
            return Ok(None);
        }
        if byte == 0x00 {
            // The frame ends only if the block was fully consumed.
            let complete = self.remaining == 0;
            self.active = false;
            if complete {
                return Ok(Some(self.pos));
            }
            self.pos = 0;
            return Err(DecodeError::InvalidFrame);
        }
        if self.remaining == 0 {
            // Code byte of the next block. The finished block implies a
            // trailing zero unless it started with `0xFF`.
            if !self.partial {
                self.put(out, 0)?;
            }
            self.start_block(byte);
            return Ok(None);
        }
        self.put(out, byte)?;
        self.remaining -= 1;
        Ok(None)
    }

    const fn start_block(&mut self, code: u8) {
        self.active = true;
        if code == 0xFF {
            self.remaining = 0xFE;
            self.partial = true;
        } else {
            self.remaining = code - 1;
            self.partial = false;
        }
    }

    fn put(&mut self, out: &mut [u8], byte: u8) -> Result<(), DecodeError> {
        let Some(slot) = out.get_mut(self.pos) else {
            return Err(DecodeError::BufferTooSmall);
        };
        *slot = byte;
        self.pos += 1;
        Ok(())
    }
}

/// An error from decoding a COBS stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum DecodeError {
    /// The frame did not fit the output buffer.
    BufferTooSmall,
    /// The frame ended in the middle of a block.
    InvalidFrame,
}
