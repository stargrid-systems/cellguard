//! Typed configuration field values.
//!
//! Each type models one register field so invalid states cannot be built. The
//! `code`/`bits`/`raw` helpers turn a value into its register bits and are only
//! visible inside the crate.

use crate::frame::CrcKind;

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
    pub(crate) const fn code(self) -> u16 {
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
    pub(crate) const fn is_ansi(self) -> bool {
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
/// ratios above 1024). The device switches to a fast-settling path
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
    pub(crate) const fn code(self) -> u16 {
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
    pub(crate) const fn code(self) -> u16 {
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
    pub(crate) const fn code(self) -> u16 {
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

    pub(crate) const fn bits(self) -> u16 {
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

    pub(crate) const fn raw(self) -> u32 {
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

    pub(crate) const fn raw(self) -> u32 {
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

    pub(crate) const fn bits(self) -> u16 {
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
    pub(crate) const fn code(self) -> u16 {
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
    pub(crate) const fn code(self) -> u16 {
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
    pub(crate) const fn code(self) -> u16 {
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
    pub(crate) const fn code(self) -> u16 {
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
    pub(crate) const fn code(self) -> u16 {
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

#[cfg(test)]
mod tests {
    use super::WordLength;

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
}
