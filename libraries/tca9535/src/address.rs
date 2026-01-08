use core::mem;
use core::ops::Range;

/// I2C slave address options for the TCA9535 device.
///
/// The address is determined by the logic levels applied to pins A2, A1, and
/// A0. This allows up to 8 different TCA9535 devices to be connected to the
/// same I2C bus.
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Address {
    /// A2=L, A1=L, A0=L (0x20)
    Lll = 0x20,
    /// A2=L, A1=L, A0=H (0x21)
    Llh = 0x21,
    /// A2=L, A1=H, A0=L (0x22)
    Lhl = 0x22,
    /// A2=L, A1=H, A0=H (0x23)
    Lhh = 0x23,
    /// A2=H, A1=L, A0=L (0x24)
    Hll = 0x24,
    /// A2=H, A1=L, A0=H (0x25)
    Hlh = 0x25,
    /// A2=H, A1=H, A0=L (0x26)
    Hhl = 0x26,
    /// A2=H, A1=H, A0=H (0x27)
    Hhh = 0x27,
}

impl Address {
    const RANGE: Range<u8> = 0x20..0x28;

    /// Creates an address from its underlying value.
    ///
    /// Returns `None` if the value does not correspond to a valid address.
    pub const fn new(value: u8) -> Option<Self> {
        if value >= Self::RANGE.start && value < Self::RANGE.end {
            // SAFETY:
            // - `Address` is `#[repr(u8)]`.
            // - Each variant of `Address` is in the `Self::RANGE`.
            // - All values in `Self::RANGE` correspond to a variant of `Address`.
            Some(unsafe { mem::transmute::<u8, Address>(value) })
        } else {
            None
        }
    }

    /// Returns the underlying address.
    pub const fn get(self) -> u8 {
        self as u8
    }
}
