//! Register addresses and the typed `ID` and `STATUS` register views.

pub const ID: u8 = 0x00;
pub const STATUS: u8 = 0x01;
pub const MODE: u8 = 0x02;
pub const CLOCK: u8 = 0x03;
pub const GAIN1: u8 = 0x04;
pub const GAIN2: u8 = 0x05;
pub const CFG: u8 = 0x06;
pub const THRESHOLD_MSB: u8 = 0x07;
pub const THRESHOLD_LSB: u8 = 0x08;
/// First per-channel register block (`CH0_CFG`). Each block is five registers.
pub const CHANNEL_BASE: u8 = 0x09;

/// Channels whose PGA gain lives in a single GAIN register (GAIN1, GAIN2).
pub const CHANNELS_PER_GAIN_REGISTER: usize = 4;

/// ID register.
#[derive(Clone, Copy, Debug)]
#[repr(transparent)]
pub struct Id(pub u16);

impl Id {
    const CHANNEL_COUNT_MASK: u16 = 0x0F00;

    /// Returns the number of channels reported by the device.
    #[must_use]
    pub const fn channel_count(self) -> u8 {
        ((self.0 & Self::CHANNEL_COUNT_MASK) >> 8) as u8
    }
}

/// Status register.
#[derive(Clone, Copy, Debug)]
#[repr(transparent)]
pub struct Status(pub u16);

impl Status {
    /// Returns true if the SPI interface is locked.
    #[must_use]
    pub const fn locked(self) -> bool {
        self.bit(15)
    }

    /// Returns true if the ADC resynchronized since the last status read.
    #[must_use]
    pub const fn resynchronized(self) -> bool {
        self.bit(14)
    }

    /// Returns true if the register map CRC changed.
    #[must_use]
    pub const fn register_map_changed(self) -> bool {
        self.bit(13)
    }

    /// Returns true if an input CRC error occurred.
    #[must_use]
    pub const fn crc_error(self) -> bool {
        self.bit(12)
    }

    /// Returns true if the CRC type is ANSI rather than CCITT.
    #[must_use]
    pub const fn crc_type_ansi(self) -> bool {
        self.bit(11)
    }

    /// Returns true if a reset occurred since the last status read.
    #[must_use]
    pub const fn reset_occurred(self) -> bool {
        self.bit(10)
    }

    /// Returns true if `channel` has new conversion data ready.
    #[must_use]
    pub const fn data_ready(self, channel: usize) -> bool {
        channel < crate::CHANNELS && self.bit(channel)
    }

    const fn bit(self, index: usize) -> bool {
        (self.0 >> index) & 1 != 0
    }
}

#[cfg(test)]
mod tests {
    use super::Status;

    #[test]
    fn status_decodes_flags_and_drdy() {
        let status = Status(0x8000 | 0x0400 | 0b0000_0101);
        assert!(status.locked());
        assert!(status.reset_occurred());
        assert!(!status.resynchronized());
        assert!(status.data_ready(0));
        assert!(!status.data_ready(1));
        assert!(status.data_ready(2));
        assert!(!status.data_ready(8));
    }
}
