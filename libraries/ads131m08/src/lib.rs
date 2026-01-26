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

use embedded_hal::spi::SpiDevice;

pub use self::error::{
    CommunicationError, CommunicationErrorKind, LockError, ResetError, WriteError,
};
pub use self::register::Status;

mod command;
mod error;
mod frame;
mod register;

/// Reset pulse width in microseconds.
///
/// `CLKIN` is between 2 and 8.2 MHz. We need to send a pulse of at least
/// 2048 cycles to reset the device.
/// `2048 @ 2 MHz = 1024 us`.
/// We round up to 1500 us to be safe.
pub const RESET_PULSE_DURATION_US: u16 = 1500;

/// Time required after a reset for the device to be ready for normal
/// operation, in microseconds.
pub const REGISTER_AQUISITION_TIME_US: u16 = 5;
const ENABLE_INPUT_CRC: bool = true;

// 24 bits is the default.
const BYTES_PER_WORD: usize = 3;
const CHANNELS: usize = 8;

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
    /// at least 5 microseconds ([`REGISTER_AQUISITION_TIME_US`]).
    pub fn reset_device_start(&mut self) -> Result<(), CommunicationError<S::Error>> {
        // As per the datasheet, a reset command must always use a full frame.
        let buf = const { frame::build_normal(command::RESET) };
        self.spi.write(&buf).map_err(CommunicationError::spi)?;
        Ok(())
    }

    /// Completes a reset operation by checking if the device has reset.
    ///
    /// See [`reset_device_start`][Self::reset_device_start] for details on the
    /// reset process.
    pub fn reset_device_complete(
        &mut self,
    ) -> Result<Result<(), ResetError>, CommunicationError<S::Error>> {
        const EXPECTED_RESPONSE: u16 = 0xFF20 | CHANNELS as u16;

        let mut buf = const { frame::build_short(command::NULL) };
        self.spi
            .transfer_in_place(&mut buf)
            .map_err(CommunicationError::spi)?;
        let response = u16::from_be_bytes([buf[0], buf[1]]);
        if response == EXPECTED_RESPONSE {
            Ok(Ok(()))
        } else {
            Ok(Err(ResetError))
        }
    }

    /// Locks the device registers.
    pub fn lock_registers(
        &mut self,
    ) -> Result<Result<(), LockError>, CommunicationError<S::Error>> {
        let buf = const { frame::build_short(command::LOCK) };
        self.spi.write(&buf).map_err(CommunicationError::spi)?;
        let status = self.read_single_register(register::STATUS)?;
        let status = Status(status);
        if status.locked() {
            Ok(Ok(()))
        } else {
            Ok(Err(LockError))
        }
    }

    /// Unlocks the device registers.
    pub fn unlock_registers(
        &mut self,
    ) -> Result<Result<(), LockError>, CommunicationError<S::Error>> {
        let buf = const { frame::build_short(command::UNLOCK) };
        self.spi.write(&buf).map_err(CommunicationError::spi)?;
        let status = self.read_single_register(register::STATUS)?;
        let status = Status(status);
        if status.locked() {
            Ok(Err(LockError))
        } else {
            Ok(Ok(()))
        }
    }

    /// Places the device into standby mode.
    ///
    /// Returns the status register corresponding to the previous operation.
    pub fn standby(&mut self) -> Result<(), CommunicationError<S::Error>> {
        let buf = const { frame::build_short(command::STANDBY) };
        self.spi.write(&buf).map_err(CommunicationError::spi)?;
        Ok(())
    }

    /// Wakes the device from standby mode to conversion mode.
    ///
    /// Returns the status register corresponding to the previous operation.
    pub fn wakeup(&mut self) -> Result<(), CommunicationError<S::Error>> {
        let buf = const { frame::build_short(command::WAKEUP) };
        self.spi.write(&buf).map_err(CommunicationError::spi)?;
        Ok(())
    }

    /// Reads conversion data from all channels into the provided array.
    pub fn read_data(
        &mut self,
        channels: &mut [i32; CHANNELS],
    ) -> Result<(), CommunicationError<S::Error>> {
        let mut buf = const { frame::build_normal(command::NULL) };
        self.spi
            .transfer_in_place(&mut buf)
            .map_err(CommunicationError::spi)?;
        let data = frame::get_verified_data(&buf)?;

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

    fn read_single_register(&mut self, addr: u8) -> Result<u16, CommunicationError<S::Error>> {
        let buf = frame::build_short(command::rreg(addr, 1));
        self.spi.write(&buf).map_err(CommunicationError::spi)?;

        let mut buf = const { frame::build_short(command::NULL) };
        self.spi
            .transfer_in_place(&mut buf)
            .map_err(CommunicationError::spi)?;
        let reg_val = u16::from_be_bytes([buf[0], buf[1]]);
        Ok(reg_val)
    }

    fn write_single_register(
        &mut self,
        addr: u8,
        value: u16,
    ) -> Result<Result<(), WriteError>, CommunicationError<S::Error>> {
        let buf = frame::build_write_one(addr, value);
        self.spi.write(&buf).map_err(CommunicationError::spi)?;
        let readback = self.read_single_register(addr)?;
        if readback == value {
            Ok(Ok(()))
        } else {
            Ok(Err(WriteError))
        }
    }
}
