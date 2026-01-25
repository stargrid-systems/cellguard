pub const ID: u8 = 0x00;
pub const STATUS: u8 = 0x01;
pub const MODE: u8 = 0x02;
pub const CLOCK: u8 = 0x03;
pub const GAIN1: u8 = 0x04;
pub const GAIN2: u8 = 0x05;
pub const CFG: u8 = 0x06;
pub const THRESHOLD_MSB: u8 = 0x07;
pub const THRESHOLD_LSB: u8 = 0x08;

const CH_BASE: u8 = 0x09;
const CH_STRIDE: u8 = 0x05;

const CH_CFG_OFFSET: u8 = 0x00;
const CH_OCAL_MSB_OFFSET: u8 = 0x01;
const CH_OCAL_LSB_OFFSET: u8 = 0x02;
const CH_GCAL_MSB_OFFSET: u8 = 0x03;
const CH_GCAL_LSB_OFFSET: u8 = 0x04;

pub const fn ch_cfg(channel: u8) -> u8 {
    ch_reg(channel, CH_CFG_OFFSET)
}

pub const fn ch_ocal_msb(channel: u8) -> u8 {
    ch_reg(channel, CH_OCAL_MSB_OFFSET)
}

pub const fn ch_gcal_msb(channel: u8) -> u8 {
    ch_reg(channel, CH_GCAL_MSB_OFFSET)
}

const fn ch_reg(channel: u8, offset: u8) -> u8 {
    debug_assert!(channel < 8, "channel out of range");
    debug_assert!(offset < CH_STRIDE, "channel register offset out of range");
    CH_BASE + channel * CH_STRIDE + offset
}

#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct Status(pub u16);

impl Status {
    const LOCK_MASK: u16 = 1 << 15;

    pub const fn locked(self) -> bool {
        (self.0 & Self::LOCK_MASK) != 0
    }
}
