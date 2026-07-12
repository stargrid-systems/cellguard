//! Streaming COBS encoder.

/// The largest number of data bytes that fit in one COBS block.
const MAX_DATA_PER_BLOCK: usize = 0xFF - 1;

/// A streaming COBS encoder.
///
/// Built from the frame to send, it yields one wire byte per [`Encoder::pull`]
/// until it returns `None`. The final `0x00` delimiter is included.
pub struct Encoder<'a> {
    state: State<'a>,
}

impl<'a> Encoder<'a> {
    /// Creates an encoder for `data`.
    #[must_use]
    pub const fn new(data: &'a [u8]) -> Self {
        Self {
            state: State::Start(data),
        }
    }

    /// Returns the next wire byte, or `None` once the frame is complete.
    pub fn pull(&mut self) -> Option<u8> {
        self.state.pull()
    }
}

enum State<'a> {
    Start(&'a [u8]),
    Block(Block<'a>),
    End,
}

impl State<'_> {
    #[expect(
        clippy::cast_possible_truncation,
        reason = "a block holds at most 254 data bytes, so the code byte fits a u8"
    )]
    fn pull(&mut self) -> Option<u8> {
        let (ret, next) = match self {
            // Emit the code byte introducing the first block.
            Self::Start(data) => {
                let block = split_first_block(data);
                (Some((block.data.len() + 1) as u8), Self::Block(block))
            }
            // This block is drained and there is no more data: emit the
            // terminator.
            Self::Block(Block {
                data: [],
                zero: false,
                rest: [],
            }) => (Some(0x00), Self::End),
            // This block is drained: emit the code byte for the next block.
            Self::Block(Block {
                data: [],
                zero: _,
                rest,
            }) => {
                let block = split_first_block(rest);
                (Some((block.data.len() + 1) as u8), Self::Block(block))
            }
            // Emit the next data byte of this block.
            Self::Block(Block {
                data: [first, tail @ ..],
                zero,
                rest,
            }) => (
                Some(*first),
                Self::Block(Block {
                    data: tail,
                    zero: *zero,
                    rest,
                }),
            ),
            Self::End => (None, Self::End),
        };
        *self = next;
        ret
    }
}

struct Block<'a> {
    data: &'a [u8],
    zero: bool,
    rest: &'a [u8],
}

#[expect(
    clippy::option_if_let_else,
    reason = "the explicit if/else reads more clearly than map_or_else here"
)]
fn split_first_block(buf: &[u8]) -> Block<'_> {
    if let Some(idx) = buf.iter().take(MAX_DATA_PER_BLOCK).position(|&b| b == 0) {
        // A zero falls within the next 254 bytes. The block runs up to it and
        // the zero itself is consumed (it is what the code byte stands in for).
        let data = buf.get(..idx).unwrap_or(&[]);
        let rest = buf.get(idx + 1..).unwrap_or(&[]);
        Block {
            data,
            zero: true,
            rest,
        }
    } else {
        // No zero in range: take a full (or final short) block with no implied
        // trailing zero.
        let len = buf.len().min(MAX_DATA_PER_BLOCK);
        let (data, rest) = buf.split_at(len);
        Block {
            data,
            zero: false,
            rest,
        }
    }
}
