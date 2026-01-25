use crate::{BYTES_PER_WORD, CHANNELS, CommunicationErrorKind, ENABLE_INPUT_CRC, command};

// command + optional CRC
const SHORT_WORDS: usize = 1 + (ENABLE_INPUT_CRC as usize);
const SHORT_BYTES: usize = SHORT_WORDS * BYTES_PER_WORD;

pub const fn build_short(command: u16) -> [u8; SHORT_BYTES] {
    let mut buf = [0; SHORT_BYTES];
    write_command_const(&mut buf, &[command]);
    buf
}

// command + value + optional CRC
const WRITE_ONE_WORDS: usize = 2 + (ENABLE_INPUT_CRC as usize);
const WRITE_ONE_BYTES: usize = WRITE_ONE_WORDS * BYTES_PER_WORD;

pub const fn build_write_one(addr: u8, value: u16) -> [u8; WRITE_ONE_BYTES] {
    let mut buf = [0; WRITE_ONE_BYTES];
    write_command_const(&mut buf, &[command::wreg(addr, 1), value]);
    buf
}

const NORMAL_WORDS: usize = 1 // command / response
        + CHANNELS // channel data
        + 1; // output CRC
const NORMAL_BYTES: usize = NORMAL_WORDS * BYTES_PER_WORD;

pub const fn build_normal(command: u16) -> [u8; NORMAL_BYTES] {
    let mut buf = [0; NORMAL_BYTES];
    write_command_const(&mut buf, &[command]);
    buf
}

/// Returns the data portion of `buf` if the CRC matches, or an error if not.
pub fn get_verified_data(buf: &[u8]) -> Result<&[u8], CommunicationErrorKind> {
    let (data, crc_word) = buf.split_at(buf.len() - BYTES_PER_WORD);
    let received_crc = u16::from_be_bytes([crc_word[0], crc_word[1]]);
    let calculated_crc = crc16_ccitt_const(data);
    if received_crc == calculated_crc {
        Ok(data)
    } else {
        Err(CommunicationErrorKind::CrcMismatch)
    }
}

/// Calculates the CRC-16-CCITT checksum for the given data.
const fn crc16_ccitt_const(data: &[u8]) -> u16 {
    const POLY: u16 = 0x1021;
    let mut crc: u16 = 0xFFFF;
    let mut byte_idx = 0;
    while byte_idx < data.len() {
        crc ^= (data[byte_idx] as u16) << 8;
        let mut bit_idx = 0;
        while bit_idx < 8 {
            if (crc & 0x8000) != 0 {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
            bit_idx += 1;
        }
        byte_idx += 1;
    }
    crc
}

const fn write_word_const(buf: &mut [u8], word_idx: usize, word: u16) {
    debug_assert!(BYTES_PER_WORD == 2 || BYTES_PER_WORD == 3 || BYTES_PER_WORD == 4);
    debug_assert!(buf.len() >= (word_idx + 1) * BYTES_PER_WORD);
    let word_bytes = word.to_be_bytes();
    let buf_offset = word_idx * BYTES_PER_WORD;
    buf[buf_offset] = word_bytes[0];
    buf[buf_offset + 1] = word_bytes[1];
    if BYTES_PER_WORD > 2 {
        buf[buf_offset + 2] = 0;
    }
    if BYTES_PER_WORD > 3 {
        buf[buf_offset + 3] = 0;
    }
}

const fn write_command_const(buf: &mut [u8], words: &[u16]) {
    let expected_len = (words.len() + ENABLE_INPUT_CRC as usize) * BYTES_PER_WORD;
    debug_assert!(buf.len() == expected_len);

    let mut word_idx = 0;
    while word_idx < words.len() {
        let word = words[word_idx];
        write_word_const(buf, word_idx, word);
        word_idx += 1;
    }

    if ENABLE_INPUT_CRC {
        let data_len = words.len() * BYTES_PER_WORD;
        let (data, remaining) = buf.split_at_mut(data_len);
        write_word_const(remaining, 0, crc16_ccitt_const(data));
    }
}
