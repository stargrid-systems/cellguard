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

pub use self::error::{Error, ErrorKind};
pub use self::register::Status;

mod command;
mod error;
mod register;

/// Reset pulse width in microseconds.
///
/// `CLKIN` is between 2 and 8.2 MHz. We need to send a pulse of at least
/// 2048 cycles to reset the device.
/// `2048 @ 2 MHz = 1024 us`.
/// We round up to 1500 us to be safe.
const RESET_PULSE_US: u16 = 1500;

/// Time required after a reset for the device to be ready for normal
/// operation, in microseconds.
pub const REGISTER_AQUISITION_TIME_US: u16 = 5;
const ENABLE_INPUT_CRC: bool = true;

// 24 bits is the default.
const BYTES_PER_WORD: usize = 3;
const CHANNELS: usize = 8;

type Ads131m08Result<T, S: SpiDevice> = Result<T, Error<S::Error>>;

pub struct Ads131m08<S> {
    spi: S,
}

impl<S: SpiDevice> Ads131m08<S> {
    /// Creates a new driver instance.
    pub const fn new(spi: S) -> Self {
        Self { spi }
    }

    /// Sends a reset command to the device.
    ///
    /// Calling this function merely sends the reset command. To confirm that
    /// the reset took place, call
    /// [`reset_device_complete`][Self::reset_device_complete] after waiting for
    /// at least 5 microseconds.
    /// Use [`REGISTER_AQUISITION_TIME_US`].
    pub fn reset_device_start(&mut self) -> Ads131m08Result<(), S> {
        // As per the datasheet, a reset command must always use a full frame.
        let buf = const { build_normal_frame(command::RESET) };
        self.spi.write(&buf).map_err(Error::spi)?;
        Ok(())
    }

    /// Completes a reset operation by checking if the device has reset.
    ///
    /// See [`reset_device_start`][Self::reset_device_start] for details on the
    /// reset process.
    pub fn reset_device_complete(&mut self) -> Ads131m08Result<(), S> {
        let mut buf = const { build_short_frame(command::NULL) };
        self.spi.transfer_in_place(&mut buf).map_err(Error::spi)?;

        todo!()
    }

    pub fn lock_registers(&mut self) -> Ads131m08Result<(), S> {
        todo!()
    }

    pub fn unlock_registers(&mut self) -> Ads131m08Result<(), S> {
        todo!()
    }

    /// Places the device into standby mode.
    ///
    /// Returns the status register corresponding to the previous operation.
    pub fn standby(&mut self) -> Ads131m08Result<(), S> {
        let buf = const { build_short_frame(command::STANDBY) };
        self.spi.write(&buf).map_err(Error::spi)?;
        Ok(())
    }

    /// Wakes the device from standby mode to conversion mode.
    ///
    /// Returns the status register corresponding to the previous operation.
    pub fn wakeup(&mut self) -> Ads131m08Result<(), S> {
        let buf = const { build_short_frame(command::WAKEUP) };
        self.spi.write(&buf).map_err(Error::spi)?;
        Ok(())
    }

    fn read_single_register(&mut self) {
        todo!()
    }

    pub fn read_adc_data(&mut self, channels: &mut [i32; CHANNELS]) -> Ads131m08Result<(), S> {
        let mut buf = const {
            let mut buf = [0u8; NORMAL_FRAME_WORDS * BYTES_PER_WORD];
            write_command_const(&mut buf, &[command::NULL]);
            buf
        };

        self.spi.transfer_in_place(&mut buf).map_err(Error::spi)?;
        let data = get_verified_data(&buf)?;

        let (_response_words, channel_words) = data.split_at(const { BYTES_PER_WORD });
        let values = channel_words.chunks_exact(BYTES_PER_WORD).map(|word| {
            let mut value = [0; 4];
            value[..BYTES_PER_WORD].copy_from_slice(word);
            i32::from_be_bytes(value) >> const { (4 - BYTES_PER_WORD) * 8 }
        });

        channels
            .iter_mut()
            .zip(values)
            .for_each(|(channel, value)| {
                *channel = value;
            });

        Ok(())
    }

    fn transfer_normal_frame<'a>(
        &mut self,
        buf: &'a mut [u8; NORMAL_FRAME_WORDS * BYTES_PER_WORD],
    ) -> Ads131m08Result<&'a [u8], S> {
        self.spi.transfer_in_place(buf).map_err(Error::spi)?;
        let data = get_verified_data(buf)?;
        Ok(data)
    }
}

/// Returns the data portion of `buf` if the CRC matches, or an error if not.
fn get_verified_data(buf: &[u8]) -> Result<&[u8], ErrorKind> {
    let (data, crc_word) = buf.split_at(buf.len() - BYTES_PER_WORD);
    let received_crc = u16::from_be_bytes([crc_word[0], crc_word[1]]);
    let calculated_crc = crc16_ccitt_const(data);
    if received_crc == calculated_crc {
        Ok(data)
    } else {
        Err(ErrorKind::CrcMismatch)
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

const SHORT_FRAME_WORDS: usize = 1 + (ENABLE_INPUT_CRC as usize);
const SHORT_FRAME_BYTES: usize = SHORT_FRAME_WORDS * BYTES_PER_WORD;

const fn build_short_frame(command: u16) -> [u8; SHORT_FRAME_BYTES] {
    let mut buf = [0; SHORT_FRAME_BYTES];
    write_command_const(&mut buf, &[command]);
    buf
}

/// The number of words in a normal frame.
const NORMAL_FRAME_WORDS: usize = 1 // command / response
        + CHANNELS // channel data
        + 1; // output CRC
const NORMAL_FRAME_BYTES: usize = NORMAL_FRAME_WORDS * BYTES_PER_WORD;

const fn build_normal_frame(command: u16) -> [u8; NORMAL_FRAME_BYTES] {
    let mut buf = [0; NORMAL_FRAME_BYTES];
    write_command_const(&mut buf, &[command]);
    buf
}
