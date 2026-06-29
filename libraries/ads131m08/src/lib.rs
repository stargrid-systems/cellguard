//! ADS131M08 driver.
//!
//! SPI Mode: Mode 1 (CPOL = 0, CPHA = 1).
//!
//! ## Lifecycle
//!
//! The driver tracks the device lifecycle in its `State` type parameter. A new
//! driver starts [`Unconfigured`]. Calling [`configure`][Ads131m08::configure]
//! moves it to [`Ready`], where conversion data can be read. This makes it a
//! compile error to read data from a device that was never configured.
//!
//! ## Data ready
//!
//! The `DRDY` pin is an active low output that indicates when new conversion
//! data are ready in conversion mode or that the requirements are met for
//! current detection when in current-detect mode. Connect the `DRDY` pin to a
//! digital input on the host to trigger periodic data retrieval in conversion
//! mode.

#![no_std]

use core::marker::PhantomData;

use embedded_hal::spi::SpiDevice;

pub use self::config::{
    ChannelConfig, Config, CrcType, DcBlock, Gain, GainCal, Mux, OffsetCal, Osr, Phase, PowerMode,
    Reference, WordLength,
};
pub use self::error::{
    CommunicationError, CommunicationErrorKind, ConfigError, LockError, ResetError, WriteError,
};
pub use self::register::{Id, Status};

mod command;
mod config;
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
pub const REGISTER_ACQUISITION_TIME_US: u16 = 5;
const CHANNELS: usize = 8;

mod sealed {
    pub trait State {}
}

/// Lifecycle state: the device has not been configured yet.
pub struct Unconfigured;

/// Lifecycle state: the device is configured and can stream conversion data.
pub struct Ready;

impl sealed::State for Unconfigured {}
impl sealed::State for Ready {}

/// Driver for the Texas Instruments ADS131M08 ADC.
///
/// The `State` type parameter tracks the device lifecycle. See the
/// [crate-level documentation](crate) for details.
pub struct Ads131m08<S, State = Unconfigured> {
    spi: S,
    format: frame::FrameFormat,
    word_length: WordLength,
    _state: PhantomData<State>,
}

impl<S: SpiDevice> Ads131m08<S, Unconfigured> {
    /// Creates a new driver instance.
    ///
    /// The driver assumes the device is in its post-reset state: 24-bit words
    /// with input CRC disabled.
    pub const fn new(spi: S) -> Self {
        Self {
            spi,
            format: frame::FrameFormat::reset_default(),
            word_length: WordLength::Bits24,
            _state: PhantomData,
        }
    }

    /// Sends a reset command to the device.
    ///
    /// Calling this function merely sends the reset command. To confirm that
    /// the reset took place, call
    /// [`reset_device_complete`][Self::reset_device_complete] after waiting for
    /// at least 5 microseconds ([`REGISTER_ACQUISITION_TIME_US`]).
    pub fn reset_device_start(&mut self) -> Result<(), CommunicationError<S::Error>> {
        // As per the datasheet, a reset command must always use a full frame.
        let mut buf = [0u8; frame::MAX_FRAME_BYTES];
        let len = frame::build(
            self.format,
            &[command::RESET],
            frame::FULL_FRAME_WORDS,
            &mut buf,
        );
        let (out, _) = buf.split_at(len);
        self.spi.write(out).map_err(CommunicationError::spi)?;
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

        let mut buf = [0u8; frame::MAX_FRAME_BYTES];
        let len = frame::build(self.format, &[command::NULL], 0, &mut buf);
        let (rx, _) = buf.split_at_mut(len);
        self.spi
            .transfer_in_place(rx)
            .map_err(CommunicationError::spi)?;
        let response = frame::read_word(rx, self.format.word_bytes(), 0);
        if response == EXPECTED_RESPONSE {
            Ok(Ok(()))
        } else {
            Ok(Err(ResetError))
        }
    }

    /// Configures the device and transitions it to [`Ready`].
    ///
    /// Writes the whole writable register block in one transaction, switches to
    /// the frame format the configuration selects, then reads the block back to
    /// confirm every register took the intended value.
    ///
    /// The register write uses the current frame format. Word length and CRC
    /// changes take effect on the following frame, so the readback uses the new
    /// format.
    pub fn configure(
        mut self,
        config: Config,
    ) -> Result<Ads131m08<S, Ready>, ConfigError<S::Error>> {
        let image = config.to_registers();

        let mut words = [0u16; 1 + frame::WRITABLE_REGISTERS];
        let [cmd, data @ ..] = &mut words;
        *cmd = command::wreg(register::MODE, reg_count(image.len()));
        data.copy_from_slice(&image);

        let mut buf = [0u8; frame::MAX_REGISTER_FRAME_BYTES];
        let len = frame::build(self.format, &words, frame::FULL_FRAME_WORDS, &mut buf);
        let (out, _) = buf.split_at(len);
        self.spi.write(out).map_err(CommunicationError::spi)?;

        self.format = config.frame_format();
        self.word_length = config.word_length;

        let mut readback = [0u16; frame::WRITABLE_REGISTERS];
        self.read_registers(register::MODE, &mut readback)?;
        if readback == image {
            Ok(Ads131m08 {
                spi: self.spi,
                format: self.format,
                word_length: self.word_length,
                _state: PhantomData,
            })
        } else {
            Err(ConfigError::Verify)
        }
    }
}

impl<S: SpiDevice> Ads131m08<S, Ready> {
    /// Locks the device registers.
    pub fn lock_registers(
        &mut self,
    ) -> Result<Result<(), LockError>, CommunicationError<S::Error>> {
        self.write_command(command::LOCK)?;
        let status = Status(self.read_single_register(register::STATUS)?);
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
        self.write_command(command::UNLOCK)?;
        let status = Status(self.read_single_register(register::STATUS)?);
        if status.locked() {
            Ok(Err(LockError))
        } else {
            Ok(Ok(()))
        }
    }

    /// Places the device into standby mode.
    pub fn standby(&mut self) -> Result<(), CommunicationError<S::Error>> {
        self.write_command(command::STANDBY)
    }

    /// Wakes the device from standby mode to conversion mode.
    pub fn wakeup(&mut self) -> Result<(), CommunicationError<S::Error>> {
        self.write_command(command::WAKEUP)
    }

    /// Sets the PGA gain for a single channel.
    ///
    /// This reads the relevant GAIN register, replaces the channel's field, and
    /// writes it back, leaving the other channels untouched.
    pub fn set_gain(
        &mut self,
        channel: usize,
        gain: Gain,
    ) -> Result<Result<(), WriteError>, CommunicationError<S::Error>> {
        debug_assert!(channel < CHANNELS, "channel out of range");
        let addr = if channel < register::CHANNELS_PER_GAIN_REGISTER {
            register::GAIN1
        } else {
            register::GAIN2
        };
        let shift = 4 * (channel % register::CHANNELS_PER_GAIN_REGISTER);
        let current = self.read_single_register(addr)?;
        let updated = (current & !(0b111 << shift)) | (gain.code() << shift);
        self.write_single_register(addr, updated)
    }

    /// Reads conversion data from all channels into the provided array and
    /// returns the [`Status`] reported alongside it.
    ///
    /// The status carries the per-channel data-ready flags, so the caller can
    /// tell which channels produced fresh samples.
    pub fn read_data(
        &mut self,
        channels: &mut [i32; CHANNELS],
    ) -> Result<Status, CommunicationError<S::Error>> {
        let mut buf = [0u8; frame::MAX_FRAME_BYTES];
        let len = frame::build(
            self.format,
            &[command::NULL],
            frame::FULL_FRAME_WORDS,
            &mut buf,
        );
        let (rx, _) = buf.split_at_mut(len);
        self.spi
            .transfer_in_place(rx)
            .map_err(CommunicationError::spi)?;
        let payload = frame::verify_output(self.format, rx)?;

        let word_bytes = self.word_length.word_bytes();
        let status = Status(frame::read_word(payload, word_bytes, 0));
        // The frame always carries one data word per channel regardless of
        // CHx_EN; disabled channels simply report stale data.
        let (_response, channel_words) = payload.split_at(word_bytes);
        for (channel, word) in channels
            .iter_mut()
            .zip(channel_words.chunks_exact(word_bytes))
        {
            *channel = self.word_length.decode_sample(word);
        }

        Ok(status)
    }

    /// Reads conversion data for the first time or after a pause in collection.
    ///
    /// The device buffers two samples per channel. After a gap, the first read
    /// returns a stale sample. This reads two frames in quick succession and
    /// returns the second, aligned one (datasheet 8.5.1.9.1). Use
    /// [`read_data`][Self::read_data] for steady-state collection.
    pub fn read_data_after_pause(
        &mut self,
        channels: &mut [i32; CHANNELS],
    ) -> Result<Status, CommunicationError<S::Error>> {
        let mut stale = [0i32; CHANNELS];
        self.read_data(&mut stale)?;
        self.read_data(channels)
    }
}

impl<S: SpiDevice, State: sealed::State> Ads131m08<S, State> {
    /// Reads the device ID register.
    pub fn read_id(&mut self) -> Result<Id, CommunicationError<S::Error>> {
        self.read_single_register(register::ID).map(Id)
    }

    /// Sends a single command in a short frame, clocking out no ADC data.
    fn write_command(&mut self, command: u16) -> Result<(), CommunicationError<S::Error>> {
        let mut buf = [0u8; frame::MAX_FRAME_BYTES];
        let len = frame::build(self.format, &[command], 0, &mut buf);
        let (out, _) = buf.split_at(len);
        self.spi.write(out).map_err(CommunicationError::spi)
    }

    fn read_single_register(&mut self, addr: u8) -> Result<u16, CommunicationError<S::Error>> {
        let mut value = [0u16; 1];
        self.read_registers(addr, &mut value)?;
        let [value] = value;
        Ok(value)
    }

    /// Reads `out.len()` consecutive registers starting at `addr`.
    ///
    /// A single-register read returns the value in the response word and the
    /// device still streams conversion data, so the frame is a full data frame
    /// (datasheet figure 8-23). A multi-register read prepends an
    /// acknowledgment word and suppresses conversion data (figure 8-24).
    fn read_registers(
        &mut self,
        addr: u8,
        out: &mut [u16],
    ) -> Result<(), CommunicationError<S::Error>> {
        let count = out.len();
        self.write_command(command::rreg(addr, reg_count(count)))?;

        let (skip, total_words) = if count == 1 {
            (0, frame::FULL_FRAME_WORDS)
        } else {
            (1, 1 + count + 1)
        };
        let mut buf = [0u8; frame::MAX_REGISTER_FRAME_BYTES];
        let len = frame::build(self.format, &[command::NULL], total_words, &mut buf);
        let (rx, _) = buf.split_at_mut(len);
        self.spi
            .transfer_in_place(rx)
            .map_err(CommunicationError::spi)?;
        frame::verify_output(self.format, rx)?;

        let word_bytes = self.format.word_bytes();
        for (i, slot) in out.iter_mut().enumerate() {
            *slot = frame::read_word(rx, word_bytes, skip + i);
        }
        Ok(())
    }

    fn write_single_register(
        &mut self,
        addr: u8,
        value: u16,
    ) -> Result<Result<(), WriteError>, CommunicationError<S::Error>> {
        self.write_registers(addr, &[value])
    }

    /// Writes `values` to consecutive registers starting at `addr`, then reads
    /// the block back to confirm the write took effect.
    fn write_registers(
        &mut self,
        addr: u8,
        values: &[u16],
    ) -> Result<Result<(), WriteError>, CommunicationError<S::Error>> {
        let count = values.len();
        let mut words = [0u16; 1 + frame::WRITABLE_REGISTERS];
        let [cmd, data @ ..] = &mut words;
        *cmd = command::wreg(addr, reg_count(count));
        let (data, _) = data.split_at_mut(count);
        data.copy_from_slice(values);

        let (frame_words, _) = words.split_at(count + 1);
        let mut buf = [0u8; frame::MAX_REGISTER_FRAME_BYTES];
        let len = frame::build(self.format, frame_words, frame::FULL_FRAME_WORDS, &mut buf);
        let (out, _) = buf.split_at(len);
        self.spi.write(out).map_err(CommunicationError::spi)?;

        let mut readback = [0u16; frame::WRITABLE_REGISTERS];
        let (readback, _) = readback.split_at_mut(count);
        self.read_registers(addr, readback)?;
        if readback == values {
            Ok(Ok(()))
        } else {
            Ok(Err(WriteError))
        }
    }
}

/// Casts a register count to the `u8` the RREG / WREG command words expect.
#[expect(
    clippy::cast_possible_truncation,
    reason = "count is bounded by WRITABLE_REGISTERS (47)"
)]
const fn reg_count(count: usize) -> u8 {
    debug_assert!(count >= 1 && count <= frame::WRITABLE_REGISTERS);
    count as u8
}
