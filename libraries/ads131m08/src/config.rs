//! Device configuration model.
//!
//! [`Config`] is a plain value type. It carries no I/O. The driver serializes
//! it into the writable register block and writes it in one transaction. See
//! [`Ads131m08::configure`][crate::Ads131m08::configure].

use crate::frame::{self, CrcKind};
use crate::{CHANNELS, register};

// The register image is a flat array; this enforces the address layout it
// assumes, namely a contiguous writable block from MODE through the channels.
const _: () = {
    assert!(register::CLOCK == register::MODE + 1);
    assert!(register::GAIN1 == register::MODE + 2);
    assert!(register::THRESHOLD_MSB == register::CFG + 1);
    assert!(register::THRESHOLD_LSB == register::CFG + 2);
    assert!(register::CHANNEL_BASE == register::THRESHOLD_LSB + 1);
};

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

/// Channel input multiplexer selection (`MUX` in `CHx_CFG`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum Mux {
    /// Normal differential analog input (default).
    #[default]
    Normal,
    /// ADC inputs shorted together, for offset measurement.
    Shorted,
    /// Positive DC test signal.
    PositiveTest,
    /// Negative DC test signal.
    NegativeTest,
}

impl Mux {
    const fn code(self) -> u16 {
        match self {
            Self::Normal => 0,
            Self::Shorted => 1,
            Self::PositiveTest => 2,
            Self::NegativeTest => 3,
        }
    }
}

/// Channel phase delay in modulator clock cycles.
///
/// Programmed into `PHASE` (10-bit two's complement), so the range is
/// `-512..=511`.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Phase(i16);

impl Phase {
    /// No phase delay.
    pub const ZERO: Self = Self(0);

    /// Creates a phase delay, clamped to the representable range.
    #[must_use]
    pub const fn new(cycles: i16) -> Self {
        let clamped = if cycles < -512 {
            -512
        } else if cycles > 511 {
            511
        } else {
            cycles
        };
        Self(clamped)
    }

    const fn bits(self) -> u16 {
        self.0.cast_unsigned() & 0x03FF
    }
}

/// Channel offset calibration in output counts.
///
/// Programmed into `OCAL` (24-bit two's complement), subtracted from each
/// conversion result.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct OffsetCal(i32);

impl OffsetCal {
    /// No offset correction (the reset value).
    pub const ZERO: Self = Self(0);

    /// Creates an offset calibration from a count value.
    #[must_use]
    pub const fn new(counts: i32) -> Self {
        Self(counts)
    }

    const fn raw(self) -> u32 {
        self.0.cast_unsigned() & 0x00FF_FFFF
    }
}

/// Channel gain calibration.
///
/// Programmed into `GCAL` (24-bit). The result is scaled by `value / 0x800000`,
/// so [`GainCal::UNITY`] applies no correction.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct GainCal(u32);

impl GainCal {
    /// Unity gain, midscale (the reset value).
    pub const UNITY: Self = Self(0x0080_0000);

    /// Creates a gain calibration from a raw 24-bit value.
    #[must_use]
    pub const fn new(raw: u32) -> Self {
        Self(raw & 0x00FF_FFFF)
    }

    const fn raw(self) -> u32 {
        self.0
    }
}

impl Default for GainCal {
    fn default() -> Self {
        Self::UNITY
    }
}

/// DC-block (high-pass) filter corner setting, programmed into `DCBLOCK`.
///
/// A higher level raises the corner frequency. Zero disables the filter.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct DcBlock(u8);

impl DcBlock {
    /// DC-block filter disabled (the reset value).
    pub const DISABLED: Self = Self(0);

    /// Creates a setting, clamped to the valid range `0..=15`.
    #[must_use]
    pub const fn new(level: u8) -> Self {
        Self(if level > 15 { 15 } else { level })
    }

    const fn bits(self) -> u16 {
        self.0 as u16
    }
}

/// Global-chop measurement delay in modulator clock periods (`GC_DLY`).
///
/// The delay is the settling time before each chopped measurement begins.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum GcDelay {
    Cycles2,
    Cycles4,
    Cycles8,
    /// 16 cycles (the reset value).
    #[default]
    Cycles16,
    Cycles32,
    Cycles64,
    Cycles128,
    Cycles256,
    Cycles512,
    Cycles1024,
    Cycles2048,
    Cycles4096,
    Cycles8192,
    Cycles16384,
    Cycles32768,
    Cycles65536,
}

impl GcDelay {
    const fn code(self) -> u16 {
        match self {
            Self::Cycles2 => 0,
            Self::Cycles4 => 1,
            Self::Cycles8 => 2,
            Self::Cycles16 => 3,
            Self::Cycles32 => 4,
            Self::Cycles64 => 5,
            Self::Cycles128 => 6,
            Self::Cycles256 => 7,
            Self::Cycles512 => 8,
            Self::Cycles1024 => 9,
            Self::Cycles2048 => 10,
            Self::Cycles4096 => 11,
            Self::Cycles8192 => 12,
            Self::Cycles16384 => 13,
            Self::Cycles32768 => 14,
            Self::Cycles65536 => 15,
        }
    }
}

/// Number of threshold exceedances required to trigger a detection (`CD_NUM`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum CdCount {
    /// One sample over threshold (default).
    #[default]
    Count1,
    Count2,
    Count4,
    Count8,
    Count16,
    Count32,
    Count64,
    Count128,
}

impl CdCount {
    const fn code(self) -> u16 {
        match self {
            Self::Count1 => 0,
            Self::Count2 => 1,
            Self::Count4 => 2,
            Self::Count8 => 3,
            Self::Count16 => 4,
            Self::Count32 => 5,
            Self::Count64 => 6,
            Self::Count128 => 7,
        }
    }
}

/// Number of samples collected per current-detect measurement (`CD_LEN`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum CdLength {
    /// 128 samples (default).
    #[default]
    Samples128,
    Samples256,
    Samples512,
    Samples768,
    Samples1280,
    Samples1792,
    Samples2560,
    Samples3584,
}

impl CdLength {
    const fn code(self) -> u16 {
        match self {
            Self::Samples128 => 0,
            Self::Samples256 => 1,
            Self::Samples512 => 2,
            Self::Samples768 => 3,
            Self::Samples1280 => 4,
            Self::Samples1792 => 5,
            Self::Samples2560 => 6,
            Self::Samples3584 => 7,
        }
    }
}

/// Current-detect mode parameters.
///
/// Programmed into the CFG and THRSHLD registers. The mode itself is entered
/// by pulsing the SYNC/RESET pin while the device is in standby, which is the
/// caller's responsibility; see [`Ads131m08::enter_current_detect`].
///
/// [`Ads131m08::enter_current_detect`]: crate::Ads131m08::enter_current_detect
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct CurrentDetectConfig {
    /// Require every enabled channel to detect, rather than any one
    /// (`CD_ALLCH`).
    pub all_channels: bool,
    /// Threshold exceedances needed to trigger.
    pub count: CdCount,
    /// Samples per measurement.
    pub length: CdLength,
    /// Comparator threshold, a 24-bit magnitude (`CD_THRSH`).
    pub threshold: u32,
}

impl CurrentDetectConfig {
    /// The CFG low byte (current-detect fields with `CD_EN` set).
    pub(crate) const fn cfg_bits(self) -> u16 {
        let all_channels = if self.all_channels { 1 << 7 } else { 0 };
        all_channels | (self.count.code() << 4) | (self.length.code() << 1) | 1
    }

    pub(crate) const fn threshold_msb(self) -> u16 {
        ((self.threshold >> 8) & 0xFFFF) as u16
    }

    /// The threshold low byte placed in the high byte of `THRSHLD_LSB`.
    pub(crate) const fn threshold_lsb_high(self) -> u16 {
        ((self.threshold & 0xFF) as u16) << 8
    }
}

/// Source that drives the `DRDY` pin (`DRDY_SEL`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DrdySource {
    /// The most lagging enabled channel (default).
    #[default]
    MostLagging,
    /// The logical OR of all enabled channels.
    LogicOr,
    /// The most leading enabled channel.
    MostLeading,
}

impl DrdySource {
    const fn code(self) -> u16 {
        match self {
            Self::MostLagging => 0b00,
            Self::LogicOr => 0b01,
            Self::MostLeading => 0b10,
        }
    }
}

/// `DRDY` signal format when conversion data is available (`DRDY_FMT`).
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum DrdyFormat {
    /// Logic low for the whole period (default).
    #[default]
    Logic,
    /// A fixed-duration low pulse.
    Pulse,
}

impl DrdyFormat {
    const fn code(self) -> u16 {
        match self {
            Self::Logic => 0,
            Self::Pulse => 1,
        }
    }
}

/// `DRDY` pin behavior in the MODE register.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub struct Drdy {
    /// Which channel drives the pin.
    pub source: DrdySource,
    /// Drive the pin high-impedance when data is not available (`DRDY_HiZ`).
    pub high_impedance: bool,
    /// Signal format when data is available.
    pub format: DrdyFormat,
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
    /// Input multiplexer selection.
    pub mux: Mux,
    /// Phase delay relative to the other channels.
    pub phase: Phase,
    /// Apply the global DC-block filter to this channel (`DCBLK_DIS` cleared).
    pub dc_block: bool,
    /// Offset calibration.
    pub offset_cal: OffsetCal,
    /// Gain calibration.
    pub gain_cal: GainCal,
}

impl ChannelConfig {
    /// The `CHx_CFG` register value for this channel.
    const fn cfg_register(self) -> u16 {
        let dcblk_dis = if self.dc_block { 0 } else { 1 << 2 };
        (self.phase.bits() << 6) | dcblk_dis | self.mux.code()
    }
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            enabled: true,
            gain: Gain::X1,
            mux: Mux::Normal,
            phase: Phase::ZERO,
            dc_block: true,
            offset_cal: OffsetCal::ZERO,
            gain_cal: GainCal::UNITY,
        }
    }
}

/// Registers per channel: CFG, `OCAL_MSB`, `OCAL_LSB`, `GCAL_MSB`, `GCAL_LSB`.
const CHANNEL_REGISTERS: usize = 5;

/// Splits a 24-bit value into its MSB register (bits 23:8) and LSB register
/// (bits 7:0 placed in the high byte, low byte reserved).
const fn split_24(raw: u32) -> (u16, u16) {
    let msb = ((raw >> 8) & 0xFFFF) as u16;
    let lsb = ((raw & 0xFF) as u16) << 8;
    (msb, lsb)
}

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
    /// Enable the register-map CRC (`REG_CRC_EN`).
    ///
    /// When set, [`Status::register_map_changed`] flags unexpected register
    /// changes.
    ///
    /// [`Status::register_map_changed`]: crate::Status::register_map_changed
    pub register_crc: bool,
    /// Enable the SPI frame timeout (`TIMEOUT`). Enabled by default.
    pub spi_timeout: bool,
    /// CRC polynomial for both input and output CRCs.
    pub crc_type: CrcType,
    /// `DRDY` pin behavior.
    pub drdy: Drdy,
    /// Modulator oversampling ratio.
    pub osr: Osr,
    /// Power mode.
    pub power_mode: PowerMode,
    /// Reference and clock source.
    pub reference: Reference,
    /// Global-chop mode. `None` disables it; `Some` enables it with a delay.
    pub global_chop: Option<GcDelay>,
    /// Global DC-block filter corner.
    pub dc_block: DcBlock,
    /// Per-channel settings.
    pub channels: [ChannelConfig; CHANNELS],
}

impl Default for Config {
    fn default() -> Self {
        Self {
            word_length: WordLength::default(),
            input_crc: false,
            register_crc: false,
            spi_timeout: true,
            crc_type: CrcType::default(),
            drdy: Drdy::default(),
            osr: Osr::default(),
            power_mode: PowerMode::default(),
            reference: Reference::default(),
            global_chop: None,
            dc_block: DcBlock::DISABLED,
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
        *cfg = self.cfg_register();
        *thr_msb = 0;
        *thr_lsb = self.dc_block.bits();

        let chunks = channels.chunks_exact_mut(CHANNEL_REGISTERS);
        for (chunk, channel) in chunks.zip(&self.channels) {
            let [ch_cfg, ocal_msb, ocal_lsb, gcal_msb, gcal_lsb] = chunk else {
                unreachable!()
            };
            *ch_cfg = channel.cfg_register();
            (*ocal_msb, *ocal_lsb) = split_24(channel.offset_cal.raw());
            (*gcal_msb, *gcal_lsb) = split_24(channel.gain_cal.raw());
        }
        regs
    }

    fn cfg_register(&self) -> u16 {
        // Default GC_DLY matches the reset value when global-chop is disabled.
        const DEFAULT_DELAY: u16 = GcDelay::Cycles16.code();
        let (delay, gc_en) = self
            .global_chop
            .map_or((DEFAULT_DELAY, 0), |delay| (delay.code(), 1 << 8));
        // Current-detect fields are added by a later task.
        (delay << 9) | gc_en
    }

    fn mode_register(&self) -> u16 {
        let mut bits = 0u16;
        if self.register_crc {
            bits |= 1 << 13;
        }
        if self.input_crc {
            bits |= 1 << 12;
        }
        bits |= u16::from(self.crc_type.is_ansi()) << 11;
        bits |= self.word_length.code() << 8;
        if self.spi_timeout {
            bits |= 1 << 4;
        }
        bits |= self.drdy.source.code() << 2;
        if self.drdy.high_impedance {
            bits |= 1 << 1;
        }
        bits |= self.drdy.format.code();
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
    use super::{
        ChannelConfig, Config, DcBlock, GainCal, GcDelay, Mux, OffsetCal, Phase, WordLength,
    };

    #[test]
    fn current_detect_config_bits() {
        use super::{CdCount, CdLength, CurrentDetectConfig};

        let config = CurrentDetectConfig {
            all_channels: false,
            count: CdCount::Count4,
            length: CdLength::Samples256,
            threshold: 0x0012_3456,
        };
        // CD_NUM=2 << 4 | CD_LEN=1 << 1 | CD_EN.
        assert_eq!(config.cfg_bits(), 0x0023);
        assert_eq!(config.threshold_msb(), 0x1234);
        assert_eq!(config.threshold_lsb_high(), 0x5600);
    }

    #[test]
    fn mode_register_encodes_drdy_timeout_and_crc() {
        use super::{Drdy, DrdyFormat, DrdySource};

        let config = Config {
            register_crc: true,
            spi_timeout: false,
            drdy: Drdy {
                source: DrdySource::MostLeading,
                high_impedance: true,
                format: DrdyFormat::Pulse,
            },
            ..Config::default()
        };
        let regs = config.to_registers();
        let (first, _) = regs.split_at(1);
        // REG_CRC_EN(13) | WLENGTH 24-bit(8) | DRDY_SEL=2(2) | DRDY_HiZ(1) |
        // DRDY_FMT(0).
        assert_eq!(first, [0x2000 | 0x0100 | 0x0008 | 0x0002 | 0x0001]);
    }

    #[test]
    fn global_chop_serializes_into_cfg() {
        let config = Config {
            global_chop: Some(GcDelay::Cycles256),
            ..Config::default()
        };
        let regs = config.to_registers();
        let (header, _) = regs.split_at(5);
        // CFG: GC_DLY code 7 << 9 | GC_EN bit 8.
        assert_eq!(header.last(), Some(&0x0F00));
    }

    #[test]
    fn channel_settings_serialize() {
        let mut config = Config {
            dc_block: DcBlock::new(5),
            ..Config::default()
        };
        let channel = ChannelConfig {
            mux: Mux::PositiveTest,
            phase: Phase::new(-1),
            dc_block: false,
            offset_cal: OffsetCal::new(-1),
            gain_cal: GainCal::new(0x0012_3456),
            ..ChannelConfig::default()
        };
        let [first, ..] = &mut config.channels;
        *first = channel;

        let regs = config.to_registers();
        let (header, channels) = regs.split_at(7);
        // THRSHLD_LSB carries the global DC-block nibble.
        assert_eq!(header.last(), Some(&0x0005));
        let (ch0, _) = channels.split_at(5);
        // CH_CFG: phase 0x3FF << 6 | DCBLK_DIS | MUX=2. OCAL=-1. GCAL=0x123456.
        assert_eq!(ch0, [0xFFC6, 0xFFFF, 0xFF00, 0x1234, 0x5600]);
    }

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
