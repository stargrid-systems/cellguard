use core::mem;
use core::ops::Range;

/// I2C device address variants for the P3T1755 temperature sensor.
///
/// The address is determined by the state of three address pins (A2, A1, A0).
/// Each variant corresponds to a specific pin configuration.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Address {
    /// Address variant 1: A2=0, A1=0, A0=SDA -> 0x40
    Addr1 = 0x40,
    /// Address variant 2: A2=0, A1=0, A0=SCL -> 0x41
    Addr2 = 0x41,
    /// Address variant 3: A2=0, A1=1, A0=SDA -> 0x42
    Addr3 = 0x42,
    /// Address variant 4: A2=0, A1=1, A0=SCL -> 0x43
    Addr4 = 0x43,
    /// Address variant 5: A2=1, A1=0, A0=SDA -> 0x44
    Addr5 = 0x44,
    /// Address variant 6: A2=1, A1=0, A0=SCL -> 0x45
    Addr6 = 0x45,
    /// Address variant 7: A2=1, A1=1, A0=SDA -> 0x46
    Addr7 = 0x46,
    /// Address variant 8: A2=1, A1=1, A0=SCL -> 0x47
    Addr8 = 0x47,
    /// Address variant 9: A2=0, A1=0, A0=0 -> 0x48
    Addr9 = 0x48,
    /// Address variant 10: A2=0, A1=0, A0=1 -> 0x49
    Addr10 = 0x49,
    /// Address variant 11: A2=0, A1=0, A0=0 -> 0x4A
    Addr11 = 0x4A,
    /// Address variant 12: A2=0, A1=0, A0=1 -> 0x4B
    Addr12 = 0x4B,
    /// Address variant 13: A2=1, A1=0, A0=0 -> 0x4C
    Addr13 = 0x4C,
    /// Address variant 14: A2=1, A1=0, A0=1 -> 0x4D
    Addr14 = 0x4D,
    /// Address variant 15: A2=1, A1=1, A0=0 -> 0x4E
    Addr15 = 0x4E,
    /// Address variant 16: A2=1, A1=1, A0=1 -> 0x4F
    Addr16 = 0x4F,
    /// Address variant 17: A2=0, A1=0, A0=SDA -> 0x50
    Addr17 = 0x50,
    /// Address variant 18: A2=0, A1=0, A0=SCL -> 0x51
    Addr18 = 0x51,
    /// Address variant 19: A2=0, A1=1, A0=SDA -> 0x52
    Addr19 = 0x52,
    /// Address variant 20: A2=0, A1=1, A0=SCL -> 0x53
    Addr20 = 0x53,
    /// Address variant 21: A2=1, A1=0, A0=SDA -> 0x54
    Addr21 = 0x54,
    /// Address variant 22: A2=1, A1=0, A0=SCL -> 0x55
    Addr22 = 0x55,
    /// Address variant 23: A2=1, A1=1, A0=SDA -> 0x56
    Addr23 = 0x56,
    /// Address variant 24: A2=1, A1=1, A0=SCL -> 0x57
    Addr24 = 0x57,
    /// Address variant 25: A2=0, A1=0, A0=0 -> 0x58
    Addr25 = 0x58,
    /// Address variant 26: A2=0, A1=0, A0=1 -> 0x59
    Addr26 = 0x59,
    /// Address variant 27: A2=0, A1=1, A0=0 -> 0x5A
    Addr27 = 0x5A,
    /// Address variant 28: A2=0, A1=1, A0=1 -> 0x5B
    Addr28 = 0x5B,
    /// Address variant 29: A2=1, A1=0, A0=0 -> 0x5C
    Addr29 = 0x5C,
    /// Address variant 30: A2=1, A1=0, A0=1 -> 0x5D
    Addr30 = 0x5D,
    /// Address variant 31: A2=1, A1=1, A0=0 -> 0x5E
    Addr31 = 0x5E,
    /// Address variant 32: A2=1, A1=1, A0=1 -> 0x5F
    Addr32 = 0x5F,
}

impl Address {
    const RANGE: Range<u8> = 0x40..0x60;

    /// Creates an address from a u8 value if it is in the valid range.
    pub const fn new(value: u8) -> Option<Self> {
        if value >= Self::RANGE.start && value < Self::RANGE.end {
            // SAFETY: Self is repr(u8) and value is in valid range.
            Some(unsafe { mem::transmute::<u8, Address>(value) })
        } else {
            None
        }
    }

    /// Returns the u8 value of the address.
    pub const fn get(self) -> u8 {
        self as u8
    }
}
