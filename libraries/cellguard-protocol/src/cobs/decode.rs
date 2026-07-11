//! Streaming COBS decoder.

/// A streaming COBS decoder.
///
/// This is a pure state machine: it owns no buffer. Feed wire bytes one at a
/// time with [`Decoder::feed`], passing the same output buffer each call. When
/// the terminating `0x00` arrives, `feed` returns the decoded frame length,
/// and the frame is in the first that many bytes of the output buffer.
///
/// Owning no buffer means a caller can hold a `Decoder` and its output buffer
/// as separate fields without a self-referential borrow.
#[derive(Debug, Clone)]
pub struct Decoder {
    pos: usize,
    state: State,
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
            state: State::Idle,
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
        match self.state.step(byte) {
            Step::Empty => Ok(Some(0)),
            Step::Continue => Ok(None),
            Step::FrameStart => {
                self.pos = 0;
                Ok(None)
            }
            Step::FrameComplete => Ok(Some(self.pos)),
            Step::Data(d) => {
                let slot = out.get_mut(self.pos).ok_or(DecodeError::BufferTooSmall)?;
                *slot = d;
                self.pos += 1;
                Ok(None)
            }
            Step::Error(err) => {
                self.state = State::Idle;
                self.pos = 0;
                Err(err)
            }
        }
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

#[derive(Debug, Clone)]
enum State {
    Idle,
    Block(u8),
    PartialBlock(u8),
}

enum Step {
    Empty,
    Continue,
    FrameStart,
    FrameComplete,
    Data(u8),
    Error(DecodeError),
}

impl State {
    // State transitions after James Munns' `cobs.rs` decoder.
    #[expect(
        clippy::match_same_arms,
        reason = "each arm documents a distinct COBS state transition even when two share a body"
    )]
    const fn step(&mut self, byte: u8) -> Step {
        let (ret, next) = match (&self, byte) {
            (Self::Idle, 0x00) => (Step::Empty, Self::Idle),
            (Self::Idle, 0xFF) => (Step::FrameStart, Self::PartialBlock(0xFE)),
            (Self::Idle, n) => (Step::FrameStart, Self::Block(n - 1)),

            (Self::Block(0), 0x00) => (Step::FrameComplete, Self::Idle),
            (Self::Block(0), 0xFF) => (Step::Data(0), Self::PartialBlock(0xFE)),
            (Self::Block(0), n) => (Step::Data(0), Self::Block(n - 1)),
            (Self::Block(_), 0) => (Step::Error(DecodeError::InvalidFrame), Self::Idle),
            (Self::Block(i), n) => (Step::Data(n), Self::Block(*i - 1)),

            (Self::PartialBlock(0), 0x00) => (Step::FrameComplete, Self::Idle),
            (Self::PartialBlock(0), 0xFF) => (Step::Continue, Self::PartialBlock(0xFE)),
            (Self::PartialBlock(0), n) => (Step::Continue, Self::Block(n - 1)),
            (Self::PartialBlock(_), 0) => (Step::Error(DecodeError::InvalidFrame), Self::Idle),
            (Self::PartialBlock(i), n) => (Step::Data(n), Self::PartialBlock(*i - 1)),
        };
        *self = next;
        ret
    }
}
