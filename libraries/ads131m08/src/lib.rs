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
//! [`enter_current_detect`][Ads131m08::<S, Ready>::enter_current_detect] arms
//! current-detect mode and yields a [`CurrentDetect`]-state driver.
//!
//! ```
//! use ads131m08::{Ads131m08, Config, ConfigError};
//! use embedded_hal::spi::SpiDevice;
//!
//! fn sample<S: SpiDevice>(spi: S) -> Result<[i32; 8], ConfigError<S::Error>> {
//!     // Configure with defaults, then start converting. On failure the driver
//!     // comes back in `err.device` so the caller can recover; here we just
//!     // forward the error.
//!     let mut device = Ads131m08::new(spi)
//!         .configure(Config::default())
//!         .map_err(|err| err.error)?;
//!     device.wakeup()?;
//!
//!     // The status carries per-channel data-ready flags.
//!     let mut channels = [0i32; 8];
//!     let _status = device.read_data(&mut channels)?;
//!     Ok(channels)
//! }
//! ```
//!
//! ## Data ready
//!
//! The `DRDY` pin is an active low output that indicates when new conversion
//! data are ready in conversion mode or that the requirements are met for
//! current detection when in current-detect mode. Connect the `DRDY` pin to a
//! digital input on the host to trigger periodic data retrieval in conversion
//! mode.
//!
//! ## Synchronization
//!
//! Conversion synchronization is driven entirely by the `SYNC/RESET` pin: the
//! host pulses it low to realign the digital filters to an external event.
//! There is no SPI command for it, so this SPI-only driver does not perform
//! synchronization itself. The host toggles the pin, and
//! [`Status::resynchronized`] reports that a resync occurred. To realign data
//! reads after a pause without the pin, see
//! [`read_data_after_pause`][Ads131m08::<S, Ready>::read_data_after_pause].

#![no_std]

use core::marker::PhantomData;

use embedded_hal::delay::DelayNs;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiDevice;

pub use self::config::{
    CdCount, CdLength, ChannelConfig, Config, CrcType, CurrentDetectConfig, DcBlock, Drdy,
    DrdyFormat, DrdySource, Gain, GainCal, GcDelay, Mux, OffsetCal, Osr, Phase, PowerMode,
    Reference, WordLength,
};
pub use self::error::{
    CommunicationError, CommunicationErrorKind, ConfigError, LockError, ResetError,
    TransitionError, WriteError,
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

/// Drives a hardware reset by pulsing the `SYNC/RESET` pin low.
///
/// Holds the pin low for [`RESET_PULSE_DURATION_US`], releases it, then waits
/// [`REGISTER_ACQUISITION_TIME_US`] for the registers to settle. This is an
/// optional convenience for callers that wire `SYNC/RESET` to a GPIO. It is
/// independent of the SPI driver, so `DRDY` and `SYNC/RESET` routing stay
/// flexible (for example a port expander or a shared interrupt line).
///
/// # Errors
///
/// Returns the pin's error if driving it fails.
pub fn pulse_reset<P: OutputPin, D: DelayNs>(reset: &mut P, delay: &mut D) -> Result<(), P::Error> {
    reset.set_low()?;
    delay.delay_us(u32::from(RESET_PULSE_DURATION_US));
    reset.set_high()?;
    delay.delay_us(u32::from(REGISTER_ACQUISITION_TIME_US));
    Ok(())
}

mod sealed {
    pub trait State {}
}

/// Lifecycle state: the device has not been configured yet.
pub struct Unconfigured;

/// Lifecycle state: the device is configured and can stream conversion data.
pub struct Ready;

/// Lifecycle state: the device is armed for current-detect mode.
pub struct CurrentDetect;

impl sealed::State for Unconfigured {}
impl sealed::State for Ready {}
impl sealed::State for CurrentDetect {}

/// Driver for the Texas Instruments ADS131M08 ADC.
///
/// The `State` type parameter tracks the device lifecycle. See the
/// [crate-level documentation](crate) for details.
pub struct Ads131m08<S, State = Unconfigured> {
    spi: S,
    format: self::frame::FrameFormat,
    word_length: WordLength,
    /// Reused byte buffer for every SPI frame. Owning it keeps the largest
    /// transfer buffer off the stack and sized for the worst case (a full
    /// register-block read or write).
    scratch: [u8; self::frame::MAX_REGISTER_FRAME_BYTES],
    _state: PhantomData<State>,
}

/// Result of a lifecycle transition.
///
/// On success the driver is in the `Next` state. On failure it is handed back
/// in its original `Prev` state inside a [`TransitionError`], so the caller
/// keeps the bus and can recover.
pub type TransitionResult<S, Next, Prev> = Result<
    Ads131m08<S, Next>,
    TransitionError<Ads131m08<S, Prev>, ConfigError<<S as embedded_hal::spi::ErrorType>::Error>>,
>;

impl<S, State> Ads131m08<S, State> {
    /// Re-tags the driver with a new lifecycle state, keeping all of its data.
    /// This is a zero-cost change of the `State` marker.
    fn with_state<Next>(self) -> Ads131m08<S, Next> {
        Ads131m08 {
            spi: self.spi,
            format: self.format,
            word_length: self.word_length,
            scratch: self.scratch,
            _state: PhantomData,
        }
    }
}

impl<S: SpiDevice> Ads131m08<S, Unconfigured> {
    /// Creates a new driver instance.
    ///
    /// The driver assumes the device is in its post-reset state: 24-bit words
    /// with input CRC disabled.
    pub const fn new(spi: S) -> Self {
        Self {
            spi,
            format: self::frame::FrameFormat::reset_default(),
            word_length: WordLength::Bits24,
            scratch: [0; self::frame::MAX_REGISTER_FRAME_BYTES],
            _state: PhantomData,
        }
    }

    /// Sends a reset command to the device.
    ///
    /// Calling this function merely sends the reset command. To confirm that
    /// the reset took place, call
    /// [`reset_device_complete`][Self::reset_device_complete] after waiting for
    /// at least 5 microseconds ([`REGISTER_ACQUISITION_TIME_US`]).
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails.
    pub fn reset_device_start(&mut self) -> Result<(), CommunicationError<S::Error>> {
        // As per the datasheet, a reset command must always use a full frame.
        let len = self::frame::build(
            self.format,
            self::command::RESET,
            &[],
            self::frame::FULL_FRAME_WORDS,
            &mut self.scratch,
        );
        let (out, _) = self.scratch.split_at(len);
        self.spi.write(out).map_err(CommunicationError::spi)?;
        Ok(())
    }

    /// Completes a reset operation by checking if the device has reset.
    ///
    /// See [`reset_device_start`][Self::reset_device_start] for details on the
    /// reset process.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails.
    pub fn reset_device_complete(&mut self) -> Result<(), ResetError<S::Error>> {
        let len = self::frame::build(
            self.format,
            self::command::NULL,
            &[],
            self::frame::FULL_FRAME_WORDS,
            &mut self.scratch,
        );
        let (rx, _) = self.scratch.split_at_mut(len);
        self.spi
            .transfer_in_place(rx)
            .map_err(CommunicationError::spi)?;
        let payload =
            self::frame::verify_output(self.format, rx).map_err(CommunicationError::from)?;
        let response = self::frame::read_word(payload, self.format.word_bytes(), 0);
        // Post-reset the device reports 0xFF20 with the channel count in the low
        // bits. Comparing in usize avoids narrowing the count to u16.
        if usize::from(response) == 0xFF20 | CHANNELS {
            Ok(())
        } else {
            Err(ResetError::NotReset)
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
    ///
    /// On failure the driver is returned unchanged in its [`Unconfigured`]
    /// state inside the [`TransitionError`], so the caller keeps the bus
    /// and can retry or reset.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails or the register block does
    /// not read back as written.
    #[expect(
        clippy::result_large_err,
        reason = "the driver is returned by value for recovery"
    )]
    pub fn configure(mut self, config: Config) -> TransitionResult<S, Ready, Unconfigured> {
        match self.try_configure(config) {
            Ok(()) => Ok(self.with_state()),
            Err(error) => Err(TransitionError::new(self, error)),
        }
    }

    fn try_configure(&mut self, config: Config) -> Result<(), ConfigError<S::Error>> {
        let image = config.to_registers();

        let cmd = self::command::wreg::<{ self::frame::WRITABLE_REGISTERS }>(self::register::MODE);
        let len = self::frame::build(
            self.format,
            cmd,
            &image,
            self::frame::FULL_FRAME_WORDS,
            &mut self.scratch,
        );
        let (out, _) = self.scratch.split_at(len);
        self.spi.write(out).map_err(CommunicationError::spi)?;

        // The write succeeded, so the device now uses the new frame format.
        // Track it for the readback. If anything fails from here the recovered
        // driver still reflects the device's actual format.
        self.format = config.frame_format();
        self.word_length = config.word_length;

        if self.registers_match(self::register::MODE, &image)? {
            Ok(())
        } else {
            Err(ConfigError::Verify)
        }
    }
}

impl<S: SpiDevice> Ads131m08<S, Ready> {
    /// Locks the device registers.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails.
    pub fn lock_registers(&mut self) -> Result<(), LockError<S::Error>> {
        self.write_command(self::command::LOCK)?;
        let status = Status(self.read_single_register(self::register::STATUS)?);
        if status.locked() {
            Ok(())
        } else {
            Err(LockError::Failed)
        }
    }

    /// Unlocks the device registers.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails.
    pub fn unlock_registers(&mut self) -> Result<(), LockError<S::Error>> {
        self.write_command(self::command::UNLOCK)?;
        let status = Status(self.read_single_register(self::register::STATUS)?);
        if status.locked() {
            Err(LockError::Failed)
        } else {
            Ok(())
        }
    }

    /// Places the device into standby mode.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails.
    pub fn standby(&mut self) -> Result<(), CommunicationError<S::Error>> {
        self.write_command(self::command::STANDBY)
    }

    /// Wakes the device from standby mode to conversion mode.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails.
    pub fn wakeup(&mut self) -> Result<(), CommunicationError<S::Error>> {
        self.write_command(self::command::WAKEUP)
    }

    /// Sets the PGA gain for a single channel.
    ///
    /// This reads the relevant GAIN register, replaces the channel's field, and
    /// writes it back, leaving the other channels untouched.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails.
    pub fn set_gain(&mut self, channel: usize, gain: Gain) -> Result<(), WriteError<S::Error>> {
        debug_assert!(channel < CHANNELS, "channel out of range");
        let addr = if channel < self::register::CHANNELS_PER_GAIN_REGISTER {
            self::register::GAIN1
        } else {
            self::register::GAIN2
        };
        let shift = 4 * (channel % self::register::CHANNELS_PER_GAIN_REGISTER);
        let current = self.read_single_register(addr)?;
        let updated = (current & !(0b111 << shift)) | (gain.code() << shift);
        self.write_single_register(addr, updated)
    }

    /// Arms current-detect mode and places the device in standby.
    ///
    /// This writes the current-detect parameters with `CD_EN` set (preserving
    /// the global-chop and DC-block settings) and issues a standby command. The
    /// device only enters current-detect mode once the host pulses the
    /// SYNC/RESET pin; a detection then drives `DRDY` low and returns the
    /// device to standby. Conversion results are not host-readable in this
    /// mode. Call [`exit`][Ads131m08::<S, CurrentDetect>::exit] to return
    /// to [`Ready`].
    ///
    /// On failure the driver is returned unchanged in its [`Ready`] state
    /// inside the [`TransitionError`], so the caller can keep streaming
    /// data or retry.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails or the registers do not read
    /// back as written.
    #[expect(
        clippy::result_large_err,
        reason = "the driver is returned by value for recovery"
    )]
    pub fn enter_current_detect(
        mut self,
        config: CurrentDetectConfig,
    ) -> TransitionResult<S, CurrentDetect, Ready> {
        match self.try_enter_current_detect(config) {
            Ok(()) => Ok(self.with_state()),
            Err(error) => Err(TransitionError::new(self, error)),
        }
    }

    fn try_enter_current_detect(
        &mut self,
        config: CurrentDetectConfig,
    ) -> Result<(), ConfigError<S::Error>> {
        let cfg = self.read_single_register(self::register::CFG)?;
        let thr_lsb = self.read_single_register(self::register::THRESHOLD_LSB)?;

        // Keep the global-chop bits (12:8); replace the current-detect byte.
        let new_cfg = (cfg & 0x1F00) | config.cfg_bits();
        // Keep the DC-block nibble (3:0); set the threshold low byte.
        let new_thr_lsb = config.threshold_lsb_high() | (thr_lsb & 0x000F);
        let block = [new_cfg, config.threshold_msb(), new_thr_lsb];

        self.write_registers(self::register::CFG, &block)?;
        self.write_command(self::command::STANDBY)?;
        Ok(())
    }

    /// Reads conversion data from all channels into the provided array and
    /// returns the [`Status`] reported alongside it.
    ///
    /// The status carries the per-channel data-ready flags, so the caller can
    /// tell which channels produced fresh samples.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails or the output CRC
    /// mismatches.
    pub fn read_data(
        &mut self,
        channels: &mut [i32; CHANNELS],
    ) -> Result<Status, CommunicationError<S::Error>> {
        let len = self::frame::build(
            self.format,
            self::command::NULL,
            &[],
            self::frame::FULL_FRAME_WORDS,
            &mut self.scratch,
        );
        let (rx, _) = self.scratch.split_at_mut(len);
        self.spi
            .transfer_in_place(rx)
            .map_err(CommunicationError::spi)?;
        let payload = self::frame::verify_output(self.format, rx)?;

        let word_bytes = self.word_length.word_bytes();
        let status = Status(self::frame::read_word(payload, word_bytes, 0));
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
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails or an output CRC mismatches.
    pub fn read_data_after_pause(
        &mut self,
        channels: &mut [i32; CHANNELS],
    ) -> Result<Status, CommunicationError<S::Error>> {
        let mut stale = [0i32; CHANNELS];
        self.read_data(&mut stale)?;
        self.read_data(channels)
    }
}

impl<S: SpiDevice> Ads131m08<S, CurrentDetect> {
    /// Disarms current-detect mode and returns the device to [`Ready`].
    ///
    /// Clears `CD_EN` and issues a wakeup command.
    ///
    /// On failure the driver is returned unchanged in its [`CurrentDetect`]
    /// state inside the [`TransitionError`], so the caller can retry.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails or the register does not
    /// read back as written.
    #[expect(
        clippy::result_large_err,
        reason = "the driver is returned by value for recovery"
    )]
    pub fn exit(mut self) -> TransitionResult<S, Ready, CurrentDetect> {
        match self.try_exit() {
            Ok(()) => Ok(self.with_state()),
            Err(error) => Err(TransitionError::new(self, error)),
        }
    }

    fn try_exit(&mut self) -> Result<(), ConfigError<S::Error>> {
        let cfg = self.read_single_register(self::register::CFG)?;
        self.write_single_register(self::register::CFG, cfg & !1)?;
        self.write_command(self::command::WAKEUP)?;
        Ok(())
    }
}

impl<S: SpiDevice, State: sealed::State> Ads131m08<S, State> {
    /// Reads the device ID register.
    ///
    /// # Errors
    ///
    /// Returns an error if SPI communication fails.
    pub fn read_id(&mut self) -> Result<Id, CommunicationError<S::Error>> {
        self.read_single_register(self::register::ID).map(Id)
    }

    /// Sends a single command in a short frame, clocking out no ADC data.
    fn write_command(&mut self, command: u16) -> Result<(), CommunicationError<S::Error>> {
        let len = self::frame::build(self.format, command, &[], 0, &mut self.scratch);
        let (out, _) = self.scratch.split_at(len);
        self.spi.write(out).map_err(CommunicationError::spi)
    }

    /// Reads `N` consecutive registers starting at `addr` into the reused
    /// scratch buffer and returns the word index of the first register value.
    ///
    /// A single-register read returns the value in the response word and the
    /// device still streams conversion data, so the frame is a full data frame
    /// (datasheet figure 8-23). A multi-register read prepends an
    /// acknowledgment word and suppresses conversion data (figure 8-24).
    fn fetch_registers<const N: usize>(
        &mut self,
        addr: u8,
    ) -> Result<usize, CommunicationError<S::Error>> {
        self.write_command(self::command::rreg::<N>(addr))?;

        let (skip, total_words) = if N == 1 {
            (0, self::frame::FULL_FRAME_WORDS)
        } else {
            (1, 1 + N + 1)
        };
        let len = self::frame::build(
            self.format,
            self::command::NULL,
            &[],
            total_words,
            &mut self.scratch,
        );
        let (rx, _) = self.scratch.split_at_mut(len);
        self.spi
            .transfer_in_place(rx)
            .map_err(CommunicationError::spi)?;
        self::frame::verify_output(self.format, rx)?;
        Ok(skip)
    }

    fn read_single_register(&mut self, addr: u8) -> Result<u16, CommunicationError<S::Error>> {
        let skip = self.fetch_registers::<1>(addr)?;
        Ok(self::frame::read_word(
            &self.scratch,
            self.format.word_bytes(),
            skip,
        ))
    }

    /// Reads `N` registers from `addr` and reports whether each one matches
    /// `expected`. Compares the readback word by word in the scratch buffer, so
    /// no separate readback array is allocated.
    fn registers_match<const N: usize>(
        &mut self,
        addr: u8,
        expected: &[u16; N],
    ) -> Result<bool, CommunicationError<S::Error>> {
        let skip = self.fetch_registers::<N>(addr)?;
        let word_bytes = self.format.word_bytes();
        let matches = expected.iter().enumerate().all(|(i, &value)| {
            self::frame::read_word(&self.scratch, word_bytes, skip + i) == value
        });
        Ok(matches)
    }

    fn write_single_register(&mut self, addr: u8, value: u16) -> Result<(), WriteError<S::Error>> {
        self.write_registers(addr, &[value])
    }

    /// Writes `N` registers to consecutive addresses starting at `addr`, then
    /// reads the block back to confirm the write took effect.
    fn write_registers<const N: usize>(
        &mut self,
        addr: u8,
        values: &[u16; N],
    ) -> Result<(), WriteError<S::Error>> {
        let cmd = self::command::wreg::<N>(addr);
        let len = self::frame::build(
            self.format,
            cmd,
            values,
            self::frame::FULL_FRAME_WORDS,
            &mut self.scratch,
        );
        let (out, _) = self.scratch.split_at(len);
        self.spi.write(out).map_err(CommunicationError::spi)?;

        if self.registers_match(addr, values)? {
            Ok(())
        } else {
            Err(WriteError::Verify)
        }
    }
}
