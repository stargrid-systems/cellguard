use core::mem::MaybeUninit;
use core::slice;

/// COBS decoder.
pub struct Decoder<'a> {
    buf: &'a mut [MaybeUninit<u8>],
    pos: usize,
    state: DecoderState,
}

impl<'a> Decoder<'a> {
    /// Creates a new decoder with the provided output buffer.
    pub const fn new_uninit(buf: &'a mut [MaybeUninit<u8>]) -> Self {
        Self {
            buf,
            pos: 0,
            state: DecoderState::new(),
        }
    }

    /// Creates a new decoder with the provided output buffer.
    pub const fn new_init(buf: &'a mut [u8]) -> Self {
        // SAFETY: `u8` and `MaybeUninit<u8>` have the same layout.
        let buf = unsafe { &mut *(buf as *mut [u8] as *mut [MaybeUninit<u8>]) };
        Self::new_uninit(buf)
    }

    /// Feeds a byte into the decoder.
    pub const fn feed(&mut self, byte: u8) -> Result<Option<usize>, DecodeError> {
        match self.state.feed(byte) {
            FeedResult::Empty => Ok(Some(0)),
            FeedResult::DataStart => {
                self.pos = 0;
                Ok(None)
            }
            FeedResult::DataComplete => Ok(Some(self.pos)),
            FeedResult::Data(d) => {
                if self.pos < self.buf.len() {
                    self.buf[self.pos].write(d);
                    self.pos += 1;
                    Ok(None)
                } else {
                    Err(DecodeError::BufferTooSmall)
                }
            }
            FeedResult::Error(err) => Err(err),
        }
    }

    /// Returns the decoded data as a byte slice.
    pub const fn data(&self) -> &[u8] {
        // SAFETY: `self.buf` holds at least `self.pos` initialized bytes.
        unsafe { slice::from_raw_parts(self.buf.as_ptr() as *const u8, self.pos) }
    }
}

pub enum DecodeError {
    /// Frame was empty.
    EmptyFrame,
    /// Frame had invalid format.
    InvalidFrame,
    /// Target buffer too small.
    BufferTooSmall,
}

enum FeedResult {
    Empty,
    DataStart,
    DataComplete,
    Data(u8),
    Error(DecodeError),
}

/// State machine for COBS decoding.
enum DecoderState {
    /// Waiting for start of frame.
    Idle,
    /// Consuming a data block (<= 254 bytes).
    Block(u8),
    /// Consuming a partial data block (255 bytes).
    PartialBlock(u8),
}

impl DecoderState {
    const fn new() -> Self {
        Self::Idle
    }

    /// Inspired by: <https://github.com/jamesmunns/cobs.rs/blob/main/src/dec.rs>
    const fn feed(&mut self, byte: u8) -> FeedResult {
        use DecoderState::*;
        use FeedResult::*;
        let (ret, state) = match (&self, byte) {
            // Currently Idle, received a terminator, ignore, stay idle
            (Idle, 0x00) => (Empty, Idle),

            // Currently Idle, received a byte indicating the
            // next 255 bytes have no zeroes, so we will have 254 unmodified
            // data bytes, then an overhead byte
            (Idle, 0xFF) => (DataStart, PartialBlock(0xFE)),

            // Currently Idle, received a byte indicating there will be a
            // zero that must be modified in the next 1..=254 bytes
            (Idle, n) => (DataStart, Block(n - 1)),

            // We have reached the end of a data run indicated by an overhead
            // byte, AND we have recieved the message terminator. This was a
            // well framed message!
            (Block(0), 0x00) => (DataComplete, Idle),

            // We have reached the end of a data run indicated by an overhead
            // byte, and the next segment of 254 bytes will have no modified
            // sentinel bytes
            (Block(0), 0xFF) => (Data(0), PartialBlock(0xFE)),

            // We have reached the end of a data run indicated by an overhead
            // byte, and we will treat this byte as a modified sentinel byte.
            // place the sentinel byte in the output, and begin processing the
            // next non-sentinel sequence
            (Block(0), n) => (Data(0), Block(n - 1)),

            // We were not expecting the sequence to terminate, but here we are.
            // Report an error due to early terminated message
            (Block(_), 0) => (Error(DecodeError::InvalidFrame), Idle),

            // We have not yet reached the end of a data run, decrement the run
            // counter, and place the byte into the decoded output
            (Block(i), n) => (Data(n), Block(*i - 1)),

            // We have reached the end of a data run indicated by an overhead
            // byte, AND we have recieved the message terminator. This was a
            // well framed message!
            (PartialBlock(0), 0x00) => (DataComplete, Idle),

            // We have reached the end of a data run, and we will begin another
            // data run with an overhead byte expected at the end
            (PartialBlock(0), 0xFF) => (Empty, PartialBlock(0xFE)),

            // We have reached the end of a data run, and we will expect `n` data
            // bytes unmodified, followed by a sentinel byte that must be modified
            (PartialBlock(0), n) => (Empty, Block(n - 1)),

            // We were not expecting the sequence to terminate, but here we are.
            // Report an error due to early terminated message
            (PartialBlock(_), 0) => (Error(DecodeError::InvalidFrame), Idle),

            // We have not yet reached the end of a data run, decrement the run
            // counter, and place the byte into the decoded output
            (PartialBlock(i), n) => (Data(n), PartialBlock(*i - 1)),
        };

        *self = state;
        ret
    }
}
