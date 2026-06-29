//! Device configuration model.
//!
//! [`Config`] is a plain value type. It carries no I/O. The driver serializes
//! it into the writable register block and writes it in one transaction. See
//! [`Ads131m08::configure`][crate::Ads131m08::configure].

use crate::frame::{self, CrcKind};
use crate::{CHANNELS, register};

/// SPI word length, programmed into `WLENGTH` of the MODE register.
///
/// Commands, responses, and registers always carry 16 bits of data MSB
/// aligned. Conversion data is 24 bits, truncated or extended to fit.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum WordLength {
    /// 16-bit words. Conversion data drops its 8 least significant bits.
    Bits16,
    /// 24-bit words (default).
    #[default]
    Bits24,
    /// 32-bit words. Conversion data is zero padded.
    Bits32ZeroPad,
    /// 32-bit words. Conversion data is sign extended.
    Bits32SignExtend,
}

impl WordLength {
    const fn code(self) -> u16 {
        match self {
            Self::Bits16 => 0b00,
            Self::Bits24 => 0b01,
            Self::Bits32ZeroPad => 0b10,
            Self::Bits32SignExtend => 0b11,
        }
    }

    pub(crate) const fn word_bytes(self) -> usize {
        match self {
            Self::Bits16 => 2,
            Self::Bits24 => 3,
            Self::Bits32ZeroPad | Self::Bits32SignExtend => 4,
        }
    }

    /// Decodes one conversion sample from its word bytes into a 24-bit-scaled
    /// signed value.
    ///
    /// In 16-bit mode the eight least significant bits are absent, so the value
    /// is the truncated 16-bit sample.
    pub(crate) fn decode_sample(self, word: &[u8]) -> i32 {
        match self {
            Self::Bits16 => match word {
                &[hi, lo, ..] => i32::from(i16::from_be_bytes([hi, lo])),
                _ => 0,
            },
            Self::Bits24 | Self::Bits32ZeroPad => match word {
                &[b0, b1, b2, ..] => i32::from_be_bytes([b0, b1, b2, 0]) >> 8,
                _ => 0,
            },
            Self::Bits32SignExtend => match word {
                &[b0, b1, b2, b3, ..] => i32::from_be_bytes([b0, b1, b2, b3]),
                _ => 0,
            },
        }
    }
}

/// CRC polynomial, programmed into `CRC_TYPE` of the MODE register.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum CrcType {
    /// 16-bit CCITT (default).
    #[default]
    Ccitt,
    /// 16-bit ANSI.
    Ansi,
}

impl CrcType {
    const fn is_ansi(self) -> bool {
        matches!(self, Self::Ansi)
    }

    pub(crate) const fn kind(self) -> CrcKind {
        match self {
            Self::Ccitt => CrcKind::Ccitt,
            Self::Ansi => CrcKind::Ansi,
        }
    }
}

/// Modulator oversampling ratio, programmed into `OSR` of the CLOCK register.
///
/// The output data rate is `fDATA = fMOD / OSR`, where `fMOD` is the modulator
/// clock set by [`PowerMode`]. A higher ratio lowers the data rate and the
/// noise. The digital filter is a SINC3 path (with an added SINC1 averager for
/// ratios above 1024); the device switches to a fast-settling path
/// automatically in global-chop mode. There is no separate filter selection.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Osr {
    Ratio128,
    Ratio256,
    Ratio512,
    /// Oversampling ratio of 1024 (default).
    #[default]
    Ratio1024,
    Ratio2048,
    Ratio4096,
    Ratio8192,
    Ratio16256,
}

impl Osr {
    const fn code(self) -> u16 {
        match self {
            Self::Ratio128 => 0,
            Self::Ratio256 => 1,
            Self::Ratio512 => 2,
            Self::Ratio1024 => 3,
            Self::Ratio2048 => 4,
            Self::Ratio4096 => 5,
            Self::Ratio8192 => 6,
            Self::Ratio16256 => 7,
        }
    }
}

/// Power mode, programmed into `PWR` of the CLOCK register.
///
/// The mode sets the modulator clock `fMOD` relative to `CLKIN`, trading power
/// for bandwidth: high-resolution runs `fMOD` fastest, very-low-power slowest.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum PowerMode {
    VeryLowPower,
    LowPower,
    /// High-resolution mode (default).
    #[default]
    HighResolution,
}

impl PowerMode {
    const fn code(self) -> u16 {
        match self {
            Self::VeryLowPower => 0b00,
            Self::LowPower => 0b01,
            Self::HighResolution => 0b10,
        }
    }
}

/// Per-channel PGA gain, programmed into the GAIN1 and GAIN2 registers.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Gain {
    /// Gain of 1 (default).
    #[default]
    X1,
    X2,
    X4,
    X8,
    X16,
    X32,
    X64,
    X128,
}

impl Gain {
    pub(crate) const fn code(self) -> u16 {
        match self {
            Self::X1 => 0,
            Self::X2 => 1,
            Self::X4 => 2,
            Self::X8 => 3,
            Self::X16 => 4,
            Self::X32 => 5,
            Self::X64 => 6,
            Self::X128 => 7,
        }
    }
}

/// Voltage reference and clock source selection in the CLOCK register.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Reference {
    /// Use an external reference instead of the internal one (`EXTREF_EN`).
    pub external: bool,
    /// Use the crystal oscillator (`XTAL_DIS` cleared).
    pub crystal: bool,
}

impl Default for Reference {
    fn default() -> Self {
        Self {
            external: false,
            crystal: true,
        }
    }
}

/// Per-channel configuration.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct ChannelConfig {
    /// Enable conversions on this channel (`CHx_EN`).
    pub enabled: bool,
    /// PGA gain for this channel.
    pub gain: Gain,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            gain: Gain::X1,
        }
    }
}

/// Reset value of the CFG register (global-chop delay 16, all detection off).
const CFG_RESET: u16 = 0x0600;
/// Reset value of each channel's `GCAL_MSB` register (unity gain, midscale).
const GCAL_MSB_RESET: u16 = 0x8000;
/// Registers per channel: CFG, `OCAL_MSB`, `OCAL_LSB`, `GCAL_MSB`, `GCAL_LSB`.
const CHANNEL_REGISTERS: usize = 5;

/// A complete device configuration.
///
/// Build one (often from [`Config::default`]), then hand it to
/// [`configure`][crate::Ads131m08::configure].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    /// SPI word length.
    pub word_length: WordLength,
    /// Require an input CRC on commands (`RX_CRC_EN`).
    pub input_crc: bool,
    /// CRC polynomial for both input and output CRCs.
    pub crc_type: CrcType,
    /// Modulator oversampling ratio.
    pub osr: Osr,
    /// Power mode.
    pub power_mode: PowerMode,
    /// Reference and clock source.
    pub reference: Reference,
    /// Per-channel settings.
    pub channels: [ChannelConfig; CHANNELS],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            word_length: WordLength::default(),
            input_crc: false,
            crc_type: CrcType::default(),
            osr: Osr::default(),
            power_mode: PowerMode::default(),
            reference: Reference::default(),
            channels: [ChannelConfig::default(); CHANNELS],
        }
    }
}

impl Config {
    /// The SPI frame format this configuration selects.
    pub(crate) const fn frame_format(&self) -> frame::FrameFormat {
        frame::FrameFormat::new(
            self.word_length.word_bytes(),
            self.input_crc,
            self.crc_type.kind(),
        )
    }

    /// Serializes the configuration into the writable register block,
    /// `02h` through `30h`.
    #[expect(
        clippy::similar_names,
        reason = "register names like gain1/gain2 and ocal/gcal are inherently paired"
    )]
    pub(crate) fn to_registers(self) -> [u16; frame::WRITABLE_REGISTERS] {
        let mut regs = [0u16; frame::WRITABLE_REGISTERS];
        let [
            mode,
            clock,
            gain1,
            gain2,
            cfg,
            thr_msb,
            thr_lsb,
            channels @ ..,
        ] = &mut regs;
        *mode = self.mode_register();
        *clock = self.clock_register();
        *gain1 = self.gain_register(0);
        *gain2 = self.gain_register(register::CHANNELS_PER_GAIN_REGISTER);
        *cfg = CFG_RESET;
        *thr_msb = 0;
        *thr_lsb = 0;

        for chunk in channels.chunks_exact_mut(CHANNEL_REGISTERS) {
            let [ch_cfg, ocal_msb, ocal_lsb, gcal_msb, gcal_lsb] = chunk else {
                unreachable!()
            };
            *ch_cfg = 0;
            *ocal_msb = 0;
            *ocal_lsb = 0;
            *gcal_msb = GCAL_MSB_RESET;
            *gcal_lsb = 0;
        }
        regs
    }

    fn mode_register(&self) -> u16 {
        let mut bits = 0u16;
        if self.input_crc {
            bits |= 1 << 12;
        }
        bits |= u16::from(self.crc_type.is_ansi()) << 11;
        bits |= self.word_length.code() << 8;
        // TIMEOUT is enabled by default; DRDY and register-map CRC are added by
        // later tasks.
        bits |= 1 << 4;
        bits
    }

    fn clock_register(&self) -> u16 {
        let mut bits = self
            .channels
            .iter()
            .enumerate()
            .fold(0u16, |bits, (i, ch)| {
                if ch.enabled {
                    bits | (1u16 << (8 + i))
                } else {
                    bits
                }
            });
        if !self.reference.crystal {
            bits |= 1 << 7;
        }
        if self.reference.external {
            bits |= 1 << 6;
        }
        bits |= self.osr.code() << 2;
        bits |= self.power_mode.code();
        bits
    }

    fn gain_register(&self, base: usize) -> u16 {
        self.channels
            .iter()
            .skip(base)
            .take(register::CHANNELS_PER_GAIN_REGISTER)
            .enumerate()
            .fold(0u16, |bits, (i, ch)| bits | (ch.gain.code() << (4 * i)))
    }
}

#[cfg(test)]
mod tests {
    use super::{Config, WordLength};

    #[test]
    fn decode_sample_handles_each_word_length() {
        assert_eq!(
            WordLength::Bits24.decode_sample(&[0x12, 0x34, 0x56]),
            0x0012_3456
        );
        assert_eq!(WordLength::Bits24.decode_sample(&[0xFF, 0xFF, 0xFF]), -1);
        assert_eq!(WordLength::Bits16.decode_sample(&[0x12, 0x34]), 0x1234);
        assert_eq!(WordLength::Bits16.decode_sample(&[0xFF, 0xFF]), -1);
        assert_eq!(
            WordLength::Bits32ZeroPad.decode_sample(&[0x12, 0x34, 0x56, 0x00]),
            0x0012_3456
        );
        assert_eq!(
            WordLength::Bits32SignExtend.decode_sample(&[0xFF, 0xFF, 0xFF, 0xFF]),
            -1
        );
    }

    #[test]
    fn default_serializes_to_reset_values() {
        let regs = Config::default().to_registers();
        // MODE with the RESET flag cleared, CLOCK, GAIN1, GAIN2, CFG,
        // THRSHLD_MSB, THRSHLD_LSB.
        let (header, channels) = regs.split_at(7);
        assert_eq!(
            header,
            [0x0110, 0xFF0E, 0x0000, 0x0000, 0x0600, 0x0000, 0x0000]
        );
        for channel in channels.chunks_exact(5) {
            assert_eq!(channel, [0x0000, 0x0000, 0x0000, 0x8000, 0x0000]);
        }
    }
}
