//! Device configuration model.
//!
//! [`Config`] is a plain value type. It carries no I/O. The driver serializes
//! it into the writable register block and writes it in one transaction. See
//! [`Ads131m08::configure`][crate::Ads131m08::configure].
//!
//! The individual typed field values live in [`fields`][self::fields].

pub use self::fields::{
    CdCount, CdLength, CrcType, DcBlock, Drdy, DrdyFormat, DrdySource, Gain, GainCal, GcDelay, Mux,
    OffsetCal, Osr, Phase, PowerMode, Reference, WordLength,
};
use crate::{CHANNELS, frame, register};

mod fields;

// The register image is a flat array. This enforces the address layout it
// assumes, namely a contiguous writable block from MODE through the channels.
const _: () = {
    assert!(register::CLOCK == register::MODE + 1);
    assert!(register::GAIN1 == register::MODE + 2);
    assert!(register::THRESHOLD_MSB == register::CFG + 1);
    assert!(register::THRESHOLD_LSB == register::CFG + 2);
    assert!(register::CHANNEL_BASE == register::THRESHOLD_LSB + 1);
};

/// Current-detect mode parameters.
///
/// Programmed into the CFG and THRSHLD registers. The mode itself is entered
/// by pulsing the SYNC/RESET pin while the device is in standby, which is the
/// caller's responsibility. See [`Ads131m08::enter_current_detect`].
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
    /// Global-chop mode. `None` disables it. `Some` enables it with a delay.
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
        // CD_EN and the current-detect fields stay clear here. They are set
        // at runtime by enter_current_detect, not as part of the static
        // config.
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
    use super::{ChannelConfig, Config, DcBlock, GainCal, GcDelay, Mux, OffsetCal, Phase};

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
