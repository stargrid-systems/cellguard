use core::mem;

/// Temperature register.
///
/// Contains two 8-bit data bytes; to store the measured Temp data.
pub const TEMP_REG: u8 = 0x00;
/// Configuration register.
///
/// Contains a single 8-bit data byte; to set the device operating condition.
pub const CONF_REG: u8 = 0x01;
/// TLOW register.
///
/// Hysteresis register, it contains two 8-bit data bytes to store the
/// hysteresis TLOW limit; default = 75 °C.
pub const T_LOW_REG: u8 = 0x02;
/// THIGH register.
///
/// Overtemperature shut down threshold register, it contains two 8-bit data
/// bytes to store the overtemperature shutdown THIGH limit; default = 80 °C.
pub const T_HIGH_REG: u8 = 0x03;

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Register {
    Temp = TEMP_REG,
    Conf = CONF_REG,
    TLow = T_LOW_REG,
    THigh = T_HIGH_REG,
}

impl Register {
    pub const fn get(self) -> u8 {
        self as u8
    }
}

#[derive(Clone, Copy)]
pub struct Config(u8);

impl Config {
    /// Default reset configuration.
    pub const RESET: Self = Self(0x28);

    /// Creates a config from a raw register value.
    pub(crate) const fn from_reg(value: u8) -> Self {
        Self(value)
    }

    /// Returns the raw register value.
    pub(crate) const fn to_reg(self) -> u8 {
        self.0
    }

    #[inline]
    const fn bit(self, bit: u8) -> bool {
        self.0 & bit != 0
    }

    #[inline]
    const fn with_bit(mut self, bit: u8, enable: bool) -> Self {
        if enable {
            self.0 |= bit;
        } else {
            self.0 &= !bit;
        }
        self
    }
}

impl Config {
    /// Shutdown Mode bit.
    const SD_BIT: u8 = 0b0000_0001;
    /// Thermostat Mode bit.
    const TM_BIT: u8 = 0b0000_0010;
    /// Polarity bit.
    const POL_BIT: u8 = 0b0000_0100;
    /// One-Shot bit.
    const OS_BIT: u8 = 0b1000_0000;

    /// Returns true if shutdown mode is enabled.
    pub const fn shutdown_mode(self) -> bool {
        self.bit(Self::SD_BIT)
    }

    /// Sets or clears the shutdown mode bit.
    pub const fn with_shutdown_mode(self, enable: bool) -> Self {
        self.with_bit(Self::SD_BIT, enable)
    }

    /// Returns true if thermostat mode is enabled.
    pub const fn thermostat_mode(self) -> bool {
        self.bit(Self::TM_BIT)
    }

    /// Sets or clears the thermostat mode bit.
    pub const fn with_thermostat_mode(self, enable: bool) -> Self {
        self.with_bit(Self::TM_BIT, enable)
    }

    /// Returns true if the polarity is active high.
    pub const fn polarity(self) -> bool {
        self.bit(Self::POL_BIT)
    }

    /// Sets or clears the polarity bit.
    pub const fn with_polarity(self, high: bool) -> Self {
        self.with_bit(Self::POL_BIT, high)
    }

    /// Returns the fault queue setting.
    pub const fn fault_queue(self) -> FaultQueue {
        FaultQueue::from_reg(self.0)
    }

    /// Sets the fault queue setting.
    pub const fn with_fault_queue(self, fq: FaultQueue) -> Self {
        let mut value = self.0 & !FaultQueue::MASK;
        value |= fq as u8;
        Self(value)
    }

    /// Returns the conversion time setting.
    pub const fn conversion_time(self) -> ConversionTime {
        ConversionTime::from_reg(self.0)
    }

    /// Sets the conversion time setting.
    pub const fn with_conversion_time(self, ct: ConversionTime) -> Self {
        let mut value = self.0 & !ConversionTime::MASK;
        value |= ct as u8;
        Self(value)
    }

    /// Returns true if one-shot mode is enabled.
    pub const fn one_shot(self) -> bool {
        self.bit(Self::OS_BIT)
    }

    /// Sets or clears the one-shot bit.
    pub const fn with_one_shot(self, enable: bool) -> Self {
        self.with_bit(Self::OS_BIT, enable)
    }
}

#[derive(Clone, Copy, Default)]
#[repr(u8)]
pub enum FaultQueue {
    One = 0b00_000,
    #[default]
    Two = 0b01_000,
    Four = 0b10_000,
    Six = 0b11_000,
}

impl FaultQueue {
    #[expect(
        clippy::unusual_byte_groupings,
        reason = "matches bit layout in datasheet"
    )]
    const MASK: u8 = 0b000_11_000;

    const fn from_reg(value: u8) -> Self {
        let bits = value & Self::MASK;
        // SAFETY: Our mask includes two bits and the enum covers all four combinations.
        unsafe { mem::transmute(bits) }
    }
}

#[derive(Clone, Copy, Default)]
#[repr(u8)]
pub enum ConversionTime {
    /// 27.5 ms
    Ms27_5 = 0b00_00000,
    /// 55 ms
    #[default]
    Ms55 = 0b01_00000,
    /// 110 ms
    Ms110 = 0b10_00000,
    /// 220 ms
    Ms220 = 0b11_00000,
}

impl ConversionTime {
    #[expect(
        clippy::unusual_byte_groupings,
        reason = "matches bit layout in datasheet"
    )]
    const MASK: u8 = 0b0_11_00000;

    const fn from_reg(value: u8) -> Self {
        let bits = value & Self::MASK;
        // SAFETY: Our mask includes two bits and the enum covers all four combinations.
        unsafe { mem::transmute(bits) }
    }
}

/// Temperature reading.
///
/// The value is internally stored in 1/16°C. The valid range is -2048 to 2047
/// representing -128.0°C to +127.9375°C.
#[derive(Clone, Copy)]
pub struct Temperature(i16);

impl Temperature {
    /// Minimum temperature (-128.0 °C).
    pub const MIN: Self = Self(-2048);
    /// Maximum temperature (+127.9375 °C).
    pub const MAX: Self = Self(2047);

    pub(crate) const fn from_regs(regs: &[u8; 2]) -> Self {
        // MSByte first (big-endian)
        let raw = i16::from_be_bytes(*regs);
        // Only the 12 MSBs are valid temperature data.
        Self(raw >> 4)
    }

    pub(crate) const fn to_regs(self) -> [u8; 2] {
        let raw = self.0 << 4;
        raw.to_be_bytes()
    }

    /// Creates a temperature from a raw value in 1/16 °C units.
    ///
    /// Returns `None` if the value is out of range.
    pub const fn from_raw(raw: i16) -> Option<Self> {
        if raw < Self::MIN.0 || raw > Self::MAX.0 {
            None
        } else {
            Some(Self(raw))
        }
    }

    /// Creates a temperature from a raw value in 1/16 °C units.
    ///
    /// Saturates to the min/max valid range if the value is out of range.
    pub const fn saturating_from_raw(raw: i16) -> Self {
        if raw < Self::MIN.0 {
            Self::MIN
        } else if raw > Self::MAX.0 {
            Self::MAX
        } else {
            Self(raw)
        }
    }

    /// Creates a temperature from degrees Celsius (°C).
    pub const fn from_degrees_celsius(deg_c: i8) -> Self {
        Self((deg_c as i16) << 4)
    }

    /// Creates a temperature from centi-degrees Celsius (1/100 °C).
    ///
    /// The resulting value is an approximation since the sensor has a
    /// resolution of 0.0625 °C. If the value is out of range, it saturates
    /// to the min/max valid range.
    pub const fn from_centi_degrees_celsius(centi_deg_c: i16) -> Self {
        // We need i32 to avoid overflow when multiplying by 16.
        let raw = (centi_deg_c as i32 * 16) / 100;
        Self::saturating_from_raw(raw as i16)
    }

    /// Returns the raw temperature value in 1/16 °C.
    pub const fn raw(self) -> i16 {
        self.0
    }

    /// Returns the temperature truncated to degrees Celsius (°C).
    pub const fn degrees_celsius(self) -> i8 {
        // The value only has 12 bits, so shifting right by 4 leaves us with exactly 8
        // bits.
        (self.0 >> 4) as i8
    }

    /// Returns the temperature in centi-degrees Celsius (1/100 °C).
    pub const fn centi_degrees_celsius(self) -> i16 {
        // We need i32 to avoid overflow when multiplying by 625.
        let centi_deg_c = (self.0 as i32 * 625) / 100;
        centi_deg_c as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn config_default_bits() {
        let c = Config::RESET;
        assert!(!c.shutdown_mode());
        assert!(!c.thermostat_mode());
        assert!(!c.polarity());
        assert!(!c.one_shot());
        assert!(matches!(c.fault_queue(), FaultQueue::Two));
        assert!(matches!(c.conversion_time(), ConversionTime::Ms55));
    }

    #[test]
    fn config_setters_bits() {
        let c = Config::RESET
            .with_shutdown_mode(true)
            .with_thermostat_mode(true)
            .with_polarity(true)
            .with_one_shot(true);
        assert!(c.shutdown_mode());
        assert!(c.thermostat_mode());
        assert!(c.polarity());
        assert!(c.one_shot());

        let c = c
            .with_shutdown_mode(false)
            .with_thermostat_mode(false)
            .with_polarity(false)
            .with_one_shot(false);
        assert!(!c.shutdown_mode());
        assert!(!c.thermostat_mode());
        assert!(!c.polarity());
        assert!(!c.one_shot());
    }

    #[test]
    fn fault_queue_variants() {
        let c = Config::RESET
            .with_fault_queue(FaultQueue::One)
            .with_fault_queue(FaultQueue::Two)
            .with_fault_queue(FaultQueue::Four)
            .with_fault_queue(FaultQueue::Six);
        assert!(matches!(c.fault_queue(), FaultQueue::Six));

        for fq in [
            FaultQueue::One,
            FaultQueue::Two,
            FaultQueue::Four,
            FaultQueue::Six,
        ] {
            let c = Config::RESET.with_fault_queue(fq);
            let got = c.fault_queue();
            assert_eq!(core::mem::discriminant(&got), core::mem::discriminant(&fq));
        }
    }

    #[test]
    fn conversion_time_variants() {
        for ct in [
            ConversionTime::Ms27_5,
            ConversionTime::Ms55,
            ConversionTime::Ms110,
            ConversionTime::Ms220,
        ] {
            let c = Config::RESET.with_conversion_time(ct);
            let got = c.conversion_time();
            assert_eq!(core::mem::discriminant(&got), core::mem::discriminant(&ct));
        }
    }

    #[test]
    fn temperature_from_raw_bounds() {
        assert!(Temperature::from_raw(Temperature::MIN.raw()).is_some());
        assert!(Temperature::from_raw(Temperature::MAX.raw()).is_some());
        assert!(Temperature::from_raw(Temperature::MIN.raw() - 1).is_none());
        assert!(Temperature::from_raw(Temperature::MAX.raw() + 1).is_none());
    }

    #[test]
    fn temperature_saturating() {
        let t = Temperature::saturating_from_raw(Temperature::MIN.raw() - 100);
        assert_eq!(t.raw(), Temperature::MIN.raw());
        let t = Temperature::saturating_from_raw(Temperature::MAX.raw() + 100);
        assert_eq!(t.raw(), Temperature::MAX.raw());
        let t = Temperature::saturating_from_raw(100);
        assert_eq!(t.raw(), 100);
    }

    #[test]
    fn temperature_roundtrip_positive() {
        let t = Temperature::from_raw(401).unwrap();
        let regs = t.to_regs();
        assert_eq!(regs, [0x19, 0x10]);
        let t2 = Temperature::from_regs(&regs);
        assert_eq!(t2.raw(), 401);
        assert_eq!(t2.degrees_celsius(), 25);
        assert_eq!(t2.centi_degrees_celsius(), 2506);
    }

    #[test]
    fn temperature_roundtrip_negative() {
        let t = Temperature::from_raw(-168).unwrap();
        let regs = t.to_regs();
        assert_eq!(regs, [0xF5, 0x80]);
        let t2 = Temperature::from_regs(&regs);
        assert_eq!(t2.raw(), -168);
        assert_eq!(t2.degrees_celsius(), -11);
        assert_eq!(t2.centi_degrees_celsius(), -1050);
    }

    #[test]
    fn from_centi_degrees_behavior() {
        let t = Temperature::from_centi_degrees_celsius(2506);
        assert_eq!(t.raw(), 400);
        let t = Temperature::from_centi_degrees_celsius(-1050);
        assert_eq!(t.raw(), -168);
        let t = Temperature::from_centi_degrees_celsius(-12800);
        assert_eq!(t.raw(), Temperature::MIN.raw());
        let t = Temperature::from_centi_degrees_celsius(12794);
        assert_eq!(t.raw(), Temperature::MAX.raw());
    }
}
