//! ADS131M08 driver.
//!
//! SPI Mode: Mode 1 (CPOL = 0, CPHA = 1).
//!
//! ## Data ready
//!
//! The `DRDY` pin is an active low output that indicates when new conversion
//! data are ready in conversion mode or that the requirements are met for
//! current detection when in current-detect mode. Connect the `DRDY` pin to a
//! digital input on the host to trigger periodic data retrieval in conversion
//! mode.

#![no_std]

use core::num::NonZeroU8;
use core::slice;

use embedded_hal::spi::{Operation, SpiDevice};

mod command;
mod register;

/// Reset pulse width in microseconds.
///
/// `CLKIN` is between 2 and 8.2 MHz. We need to send a pulse of at least
/// 2048 cycles to reset the device.
/// `2048 @ 2 MHz = 1024 us`.
/// We round up to 1500 us to be safe.
const RESET_PULSE_US: u16 = 1500;
const ENABLE_INPUT_CRC: bool = true;

const BYTES_PER_WORD: usize = 3;
const CHANNELS: usize = 8;

pub struct Ads131m08<S, P> {
    spi: S,
    sync_reset: P,
}

impl<S: SpiDevice> Ads131m08<S, ()> {
    pub const fn new(device: S) -> Self {
        // Self { device }
        todo!()
    }

    pub fn init(&mut self) {
        // 1. reset using SYNC/RESET pin
        // 2. validate initial null command
        // 2. disable all channels
        // 3.
    }

    pub fn read_adc_data(&mut self) -> Result<[i32; CHANNELS], S::Error> {
        const WORDS: usize = 1 // command
        + CHANNELS // channel data
        + 1; // output CRC

        let mut buf = const {
            let mut buf = [0u8; WORDS * BYTES_PER_WORD];
            write_command_const(&mut buf, &[command::NULL]);
            buf
        };

        self.spi.transfer_in_place(&mut buf)?;

        // the first word is the response
        // followed by CHANNELS words of channel data
        // followed by the CRC word

        todo!()
    }
}

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
    assert!(BYTES_PER_WORD == 2 || BYTES_PER_WORD == 3 || BYTES_PER_WORD == 4);
    assert!(buf.len() >= (word_idx + 1) * BYTES_PER_WORD);
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
    assert!(buf.len() == expected_len);

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
