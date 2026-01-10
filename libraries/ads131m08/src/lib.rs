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

use embedded_hal::spi::SpiDevice;

mod command;
mod register;

/// Reset pulse width in microseconds.
///
/// `CLKIN` is between 2 and 8.2 MHz. We need to send a pulse of at least
/// 2048 cycles to reset the device.
/// `2048 @ 2 MHz = 1024 us`.
/// We round up to 1500 us to be safe.
const RESET_PULSE_US: u16 = 1500;
const ENABLE_CRC: bool = true;

pub struct Ads131m08<SpiDevice, SyncReset> {
    device: SpiDevice,
    sync_reset: SyncReset,
}

impl<SpiDev: SpiDevice> Ads131m08<SpiDev, ()> {
    pub const fn new(device: SpiDev) -> Self {
        // Self { device }
        todo!()
    }

    pub fn init(&mut self) {
        // 1. reset using SYNC/RESET pin
        // 2. validate initial null command
        // 2. disable all channels
        // 3.
    }

    pub fn read(&mut self, addr: u8, count: NonZeroU8) {
        todo!()
    }
}

const fn write_buf_const(buf: &mut [u8], words: &[u16], bytes_per_word: u8) {
    assert!(bytes_per_word == 2 || bytes_per_word == 3 || bytes_per_word == 4);
    let expected_len = (words.len() + ENABLE_CRC as usize) * (bytes_per_word as usize);
    assert!(buf.len() == expected_len);

    let mut word_idx = 0;
    while word_idx < words.len() {
        let word = words[word_idx];
        let word_bytes = word.to_be_bytes();
        let buf_offset = word_idx * (bytes_per_word as usize);
        buf[buf_offset] = word_bytes[0];
        buf[buf_offset + 1] = word_bytes[1];
        word_idx += 1;
    }

    if ENABLE_CRC {
        todo!()
    }
}
