use crate::{CHANNELS, CommunicationErrorKind};

const MAX_WORD_BYTES: usize = 4;

/// Words in a standard full frame: the response word, one word per channel,
/// and the trailing output CRC word.
pub const FULL_FRAME_WORDS: usize = 1 + CHANNELS + 1;

/// Worst-case length of a standard full frame in bytes (32-bit words).
pub const MAX_FRAME_BYTES: usize = FULL_FRAME_WORDS * MAX_WORD_BYTES;

/// Number of registers in the writable block, `02h` through `30h`.
pub const WRITABLE_REGISTERS: usize = 47;

/// Worst-case length of a register block transfer in bytes: a command or
/// acknowledgment word, every writable register, and the output CRC word.
pub const MAX_REGISTER_FRAME_BYTES: usize = (1 + WRITABLE_REGISTERS + 1) * MAX_WORD_BYTES;

/// CRC polynomial, selected by `CRC_TYPE` in the MODE register.
///
/// Both types use a seed of `0xFFFF`.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum CrcKind {
    Ccitt,
    Ansi,
}

impl CrcKind {
    const fn polynomial(self) -> u16 {
        match self {
            Self::Ccitt => 0x1021,
            Self::Ansi => 0x8005,
        }
    }
}

/// Runtime SPI frame format.
///
/// Mirrors the device's current WLENGTH, `RX_CRC_EN`, and `CRC_TYPE` settings
/// so host frames always match what the device expects.
#[derive(Clone, Copy)]
pub struct FrameFormat {
    word_bytes: usize,
    input_crc: bool,
    crc: CrcKind,
}

impl FrameFormat {
    /// Format right after a power-on or command reset: 24-bit words, input CRC
    /// disabled, CCITT polynomial.
    pub const fn reset_default() -> Self {
        Self {
            word_bytes: 3,
            input_crc: false,
            crc: CrcKind::Ccitt,
        }
    }

    /// Builds a format from configured word length, input CRC, and CRC type.
    pub const fn new(word_bytes: usize, input_crc: bool, crc: CrcKind) -> Self {
        Self {
            word_bytes,
            input_crc,
            crc,
        }
    }

    pub const fn word_bytes(self) -> usize {
        self.word_bytes
    }
}

/// Builds an input frame into `buf` and returns the number of bytes to send.
///
/// `words` holds the logical 16-bit words: the command followed by any data.
/// When the format enables input CRC, a CRC word is appended over those words.
/// The frame is then zero-padded to at least `min_words` total words so the
/// host clocks out all of the device's response.
pub fn build(fmt: FrameFormat, words: &[u16], min_words: usize, buf: &mut [u8]) -> usize {
    for (idx, &word) in words.iter().enumerate() {
        write_word(buf, fmt.word_bytes, idx, word);
    }

    let mut written = words.len();
    if fmt.input_crc {
        let (data, _) = buf.split_at(written * fmt.word_bytes);
        let crc = crc16(fmt.crc, data);
        write_word(buf, fmt.word_bytes, written, crc);
        written += 1;
    }

    let total = if written > min_words {
        written
    } else {
        min_words
    };
    zero_words(buf, fmt.word_bytes, written, total);
    total * fmt.word_bytes
}

/// Verifies the output CRC of a received `frame` and returns the payload, that
/// is every word except the trailing CRC word.
pub fn verify_output(fmt: FrameFormat, frame: &[u8]) -> Result<&[u8], CommunicationErrorKind> {
    let (payload, crc_word) = frame.split_at(frame.len() - fmt.word_bytes);
    let &[hi, lo, ..] = crc_word else {
        return Err(CommunicationErrorKind::CrcMismatch);
    };
    let received = u16::from_be_bytes([hi, lo]);
    if received == crc16(fmt.crc, payload) {
        Ok(payload)
    } else {
        Err(CommunicationErrorKind::CrcMismatch)
    }
}

/// Reads the 16-bit value held in word `idx` of `frame`.
///
/// Commands, responses, and register contents are 16 bits MSB-aligned, so only
/// the first two bytes of the word carry data.
pub fn read_word(frame: &[u8], word_bytes: usize, idx: usize) -> u16 {
    let (_, rest) = frame.split_at(idx * word_bytes);
    let &[hi, lo, ..] = rest else {
        return 0;
    };
    u16::from_be_bytes([hi, lo])
}

/// Writes a 16-bit value MSB-aligned into word `idx`, zeroing the padding
/// bytes.
fn write_word(buf: &mut [u8], word_bytes: usize, idx: usize, value: u16) {
    let (_, rest) = buf.split_at_mut(idx * word_bytes);
    let (word, _) = rest.split_at_mut(word_bytes);
    let (high, low) = word.split_at_mut(2);
    high.copy_from_slice(&value.to_be_bytes());
    low.fill(0);
}

/// Zeroes the words in the half-open range `[from, to)`.
fn zero_words(buf: &mut [u8], word_bytes: usize, from: usize, to: usize) {
    if to <= from {
        return;
    }
    let (_, rest) = buf.split_at_mut(from * word_bytes);
    let (pad, _) = rest.split_at_mut((to - from) * word_bytes);
    pad.fill(0);
}

/// Computes the 16-bit CRC over `data` using the selected polynomial.
fn crc16(kind: CrcKind, data: &[u8]) -> u16 {
    let poly = kind.polynomial();
    data.iter().fold(0xFFFF, |seed, &byte| {
        let mut crc = seed ^ (u16::from(byte) << 8);
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 {
                (crc << 1) ^ poly
            } else {
                crc << 1
            };
        }
        crc
    })
}

#[cfg(test)]
mod tests {
    use super::{CrcKind, FrameFormat, MAX_FRAME_BYTES, build, crc16, verify_output};

    const PLAIN: FrameFormat = FrameFormat::reset_default();
    const WITH_CRC: FrameFormat = FrameFormat {
        word_bytes: 3,
        input_crc: true,
        crc: CrcKind::Ccitt,
    };

    #[test]
    fn crc_seed_for_empty_input() {
        assert_eq!(crc16(CrcKind::Ccitt, &[]), 0xFFFF);
        assert_eq!(crc16(CrcKind::Ansi, &[]), 0xFFFF);
    }

    #[test]
    fn build_writes_msb_aligned_word_with_padding() {
        let mut buf = [0xAA; MAX_FRAME_BYTES];
        let len = build(PLAIN, &[0x1234], 0, &mut buf);
        assert_eq!(len, 3);
        let (frame, _) = buf.split_at(len);
        assert_eq!(frame, [0x12, 0x34, 0x00]);
    }

    #[test]
    fn build_pads_to_full_frame() {
        let mut buf = [0xAA; MAX_FRAME_BYTES];
        let len = build(PLAIN, &[0x0011], 10, &mut buf);
        assert_eq!(len, 30);
        let (frame, _) = buf.split_at(len);
        let (first, rest) = frame.split_at(3);
        assert_eq!(first, [0x00, 0x11, 0x00]);
        assert!(rest.iter().all(|&b| b == 0));
    }

    #[test]
    fn build_appends_input_crc_word() {
        let mut buf = [0; MAX_FRAME_BYTES];
        let len = build(WITH_CRC, &[0x1234], 0, &mut buf);
        assert_eq!(len, 6);
        let (frame, _) = buf.split_at(len);
        assert!(verify_output(WITH_CRC, frame).is_ok());
    }

    #[test]
    fn verify_output_detects_corruption() {
        let mut buf = [0; MAX_FRAME_BYTES];
        let len = build(WITH_CRC, &[0x1234], 0, &mut buf);
        let [first, ..] = &mut buf;
        *first ^= 0xFF;
        let (frame, _) = buf.split_at(len);
        assert!(verify_output(WITH_CRC, frame).is_err());
    }
}
