pub struct Encoder<'a> {
    state: EncoderState<'a>,
}

impl<'a> Encoder<'a> {
    /// Creates a new COBS encoder with the provided input data.
    pub const fn new(data: &'a [u8]) -> Self {
        Self {
            state: EncoderState::new(data),
        }
    }

    pub fn pull(&mut self) -> Option<u8> {
        self.state.pull()
    }
}

enum EncoderState<'a> {
    Start(&'a [u8]),
    Block(Block<'a>),
    End,
}

impl<'a> EncoderState<'a> {
    const fn new(data: &'a [u8]) -> Self {
        Self::Start(data)
    }

    fn pull(&mut self) -> Option<u8> {
        let (ret, state) = match self {
            // Not started yet, emit the code byte for the first block.
            Self::Start(data) => {
                let block = split_first_block(data);
                (Some((block.data.len() + 1) as u8), Self::Block(block))
            }
            // We exhausted all the data, emit the final zero byte.
            Self::Block(Block {
                data: [],
                zero: false,
                rest: [],
            }) => (Some(0x00), Self::End),
            // We exhausted this block, emit the code byte for the next block.
            Self::Block(Block {
                data: [],
                zero: _,
                rest,
            }) => {
                let block = split_first_block(rest);
                (Some((block.data.len() + 1) as u8), Self::Block(block))
            }
            // We have data in this block, emit the next byte.
            Self::Block(Block {
                data: [first, data @ ..],
                zero,
                rest,
            }) => (
                Some(*first),
                Self::Block(Block {
                    data,
                    zero: *zero,
                    rest,
                }),
            ),
            // We're done.
            Self::End => (None, Self::End),
        };
        *self = state;
        ret
    }
}

struct Block<'a> {
    data: &'a [u8],
    zero: bool,
    rest: &'a [u8],
}

fn split_first_block(buf: &[u8]) -> Block<'_> {
    const MAX_DATA_PER_BLOCK: usize = 0xFF - 1;
    if let Some(idx) = buf.iter().take(MAX_DATA_PER_BLOCK).position(|&b| b == 0) {
        // There's a zero in the next 254 bytes.
        // The next block is right up to (but not including) the zero.
        let data = &buf[..idx];
        let rest = &buf.get(idx + 1..).unwrap_or(&[]);
        Block {
            data,
            zero: true,
            rest,
        }
    } else {
        // There's no zero in the next 254 bytes.
        let len = buf.len().min(MAX_DATA_PER_BLOCK);
        let (data, rest) = buf.split_at(len);
        Block {
            data,
            zero: false,
            rest,
        }
    }
}
