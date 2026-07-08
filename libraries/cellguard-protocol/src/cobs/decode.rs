//! Streaming COBS decoder.

/// A streaming COBS decoder.
///
/// Feed wire bytes one at a time with [`Decoder::feed`]. It writes decoded bytes
/// into the buffer given at construction and reports the frame length when the
/// terminating `0x00` arrives. Read the frame with [`Decoder::data`].
pub struct Decoder<'a> {
    buf: &'a mut [u8],
    pos: usize,
    state: State,
}

impl<'a> Decoder<'a> {
    /// Creates a decoder writing into `buf`.
    pub const fn new(buf: &'a mut [u8]) -> Self {
        Self {
            buf,
            pos: 0,
            state: State::Idle,
        }
    }

    /// Feeds one wire byte.
    ///
    /// Returns `Ok(Some(len))` when a complete frame has been decoded (its bytes
    /// are then available from [`Decoder::data`]), or `Ok(None)` while a frame is
    /// still in progress.
    ///
    /// # Errors
    ///
    /// Returns [`DecodeError::BufferTooSmall`] if the frame does not fit the
    /// output buffer, or [`DecodeError::InvalidFrame`] on a malformed frame.
    pub fn feed(&mut self, byte: u8) -> Result<Option<usize>, DecodeError> {
        match self.state.step(byte) {
            Step::Empty => Ok(Some(0)),
            Step::FrameStart => {
                self.pos = 0;
                Ok(None)
            }
            Step::FrameComplete => Ok(Some(self.pos)),
            Step::Data(d) => {
                let slot = self.buf.get_mut(self.pos).ok_or(DecodeError::BufferTooSmall)?;
                *slot = d;
                self.pos += 1;
                Ok(None)
            }
            Step::Error(err) => {
                self.state = State::Idle;
                Err(err)
            }
        }
    }

    /// Returns the bytes decoded so far for the current or last frame.
    #[must_use]
    pub fn data(&self) -> &[u8] {
        self.buf.get(..self.pos).unwrap_or(&[])
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

enum State {
    Idle,
    Block(u8),
    PartialBlock(u8),
}

enum Step {
    Empty,
    FrameStart,
    FrameComplete,
    Data(u8),
    Error(DecodeError),
}

impl State {
    // State transitions after James Munns' `cobs.rs` decoder.
    #[expect(
        clippy::match_same_arms,
        reason = "each arm documents a distinct COBS state transition even when \
                  two share a body"
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
            (Self::PartialBlock(0), 0xFF) => (Step::Empty, Self::PartialBlock(0xFE)),
            (Self::PartialBlock(0), n) => (Step::Empty, Self::Block(n - 1)),
            (Self::PartialBlock(_), 0) => (Step::Error(DecodeError::InvalidFrame), Self::Idle),
            (Self::PartialBlock(i), n) => (Step::Data(n), Self::PartialBlock(*i - 1)),
        };
        *self = next;
        ret
    }
}
