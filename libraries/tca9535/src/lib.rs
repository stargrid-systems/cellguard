//! Low level driver for the TCA9535 I2C I/O expander.
//!
//! This library currently opts to provide only a low level interface to the
//! TCA9535 device. Higher level abstractions like abstracting individual pins
//! as types implementing the `embedded-hal` traits are not zero-cost.

#![no_std]
// Set per-crate so it lints the library only, not the test crates.
#![warn(missing_docs)]

use core::mem;
use core::ops::Range;

use embedded_hal::i2c::I2c;

const INPUT_PORT0: u8 = 0x00;
const OUTPUT_PORT0: u8 = 0x02;
const POLARITY_INVERSION_PORT0: u8 = 0x04;
const CONFIGURATION_PORT0: u8 = 0x06;

/// Low level TCA9535 device driver.
pub struct Tca9535<I> {
    i2c: I,
    addr: Address,
}

impl<I: I2c> Tca9535<I> {
    /// Creates a new driver instance.
    pub const fn new(i2c: I, addr: Address) -> Self {
        Self { i2c, addr }
    }

    /// Releases the I2C bus from the driver.
    pub fn into_inner(self) -> I {
        self.i2c
    }

    /// Reads the input registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C transaction fails.
    pub fn read_input(&mut self) -> Result<Input, I::Error> {
        self.read_register_pair(INPUT_PORT0).map(Input)
    }

    /// Reads the output registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C transaction fails.
    pub fn read_output(&mut self) -> Result<Output, I::Error> {
        self.read_register_pair(OUTPUT_PORT0).map(Output)
    }

    /// Writes the output registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C transaction fails.
    pub fn write_output(&mut self, value: Output) -> Result<(), I::Error> {
        self.write_register_pair(OUTPUT_PORT0, value.0)
    }

    /// Reads the polarity inversion registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C transaction fails.
    pub fn read_polarity_inversion(&mut self) -> Result<PolarityInversion, I::Error> {
        self.read_register_pair(POLARITY_INVERSION_PORT0)
            .map(PolarityInversion)
    }

    /// Writes the polarity inversion registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C transaction fails.
    pub fn write_polarity_inversion(&mut self, value: PolarityInversion) -> Result<(), I::Error> {
        self.write_register_pair(POLARITY_INVERSION_PORT0, value.0)
    }

    /// Reads the configuration registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C transaction fails.
    pub fn read_configuration(&mut self) -> Result<Configuration, I::Error> {
        self.read_register_pair(CONFIGURATION_PORT0)
            .map(Configuration)
    }

    /// Writes the configuration registers.
    ///
    /// # Errors
    ///
    /// Returns an error if the I2C transaction fails.
    pub fn write_configuration(&mut self, value: Configuration) -> Result<(), I::Error> {
        self.write_register_pair(CONFIGURATION_PORT0, value.0)
    }

    fn read_register_pair(&mut self, start: u8) -> Result<u16, I::Error> {
        let mut buf = [0u8; 2];
        self.i2c.write_read(self.addr.get(), &[start], &mut buf)?;
        // LSB first
        Ok(u16::from_le_bytes(buf))
    }

    fn write_register_pair(&mut self, start: u8, value: u16) -> Result<(), I::Error> {
        let [b0, b1] = value.to_le_bytes();
        self.i2c.write(self.addr.get(), &[start, b0, b1])
    }
}

/// I2C slave address options for the TCA9535 device.
///
/// The address is determined by the logic levels applied to pins A2, A1, and
/// A0. This allows up to 8 different TCA9535 devices to be connected to the
/// same I2C bus.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
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
    #[must_use]
    pub const fn new(value: u8) -> Option<Self> {
        if value >= Self::RANGE.start && value < Self::RANGE.end {
            // SAFETY:
            // - `Address` is `#[repr(u8)]`.
            // - Each variant of `Address` is in the `Self::RANGE`.
            // - All values in `Self::RANGE` correspond to a variant of `Address`.
            Some(unsafe { mem::transmute::<u8, Self>(value) })
        } else {
            None
        }
    }

    /// Returns the underlying address.
    #[must_use]
    pub const fn get(self) -> u8 {
        self as u8
    }
}

/// Bit-index for a TCA9535 pin.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PinIndex {
    /// Pin 0 (port 0, bit 0).
    P0,
    /// Pin 1 (port 0, bit 1).
    P1,
    /// Pin 2 (port 0, bit 2).
    P2,
    /// Pin 3 (port 0, bit 3).
    P3,
    /// Pin 4 (port 0, bit 4).
    P4,
    /// Pin 5 (port 0, bit 5).
    P5,
    /// Pin 6 (port 0, bit 6).
    P6,
    /// Pin 7 (port 0, bit 7).
    P7,
    /// Pin 8 (port 1, bit 0).
    P8,
    /// Pin 9 (port 1, bit 1).
    P9,
    /// Pin 10 (port 1, bit 2).
    P10,
    /// Pin 11 (port 1, bit 3).
    P11,
    /// Pin 12 (port 1, bit 4).
    P12,
    /// Pin 13 (port 1, bit 5).
    P13,
    /// Pin 14 (port 1, bit 6).
    P14,
    /// Pin 15 (port 1, bit 7).
    P15,
}

impl PinIndex {
    /// Returns the bit position of this pin (0-15).
    #[must_use]
    pub const fn bit(self) -> u8 {
        match self {
            Self::P0 => 0,
            Self::P1 => 1,
            Self::P2 => 2,
            Self::P3 => 3,
            Self::P4 => 4,
            Self::P5 => 5,
            Self::P6 => 6,
            Self::P7 => 7,
            Self::P8 => 8,
            Self::P9 => 9,
            Self::P10 => 10,
            Self::P11 => 11,
            Self::P12 => 12,
            Self::P13 => 13,
            Self::P14 => 14,
            Self::P15 => 15,
        }
    }

    /// Returns a bitmask with only this pin set.
    #[must_use]
    pub const fn mask(self) -> u16 {
        1 << self.bit()
    }
}

/// Input registers.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Input(pub u16);

impl Input {
    /// Returns true if the specified pin is high.
    #[must_use]
    pub const fn is_high(self, pin: PinIndex) -> bool {
        self.0 & pin.mask() != 0
    }

    /// Returns true if the specified pin is low.
    #[must_use]
    pub const fn is_low(self, pin: PinIndex) -> bool {
        self.0 & pin.mask() == 0
    }
}

/// Output registers.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[must_use]
pub struct Output(pub u16);

impl Output {
    /// Returns a new value with the specified pin set high.
    pub const fn with_high(mut self, pin: PinIndex) -> Self {
        self.0 |= pin.mask();
        self
    }

    /// Returns a new value with the specified pin set low.
    pub const fn with_low(mut self, pin: PinIndex) -> Self {
        self.0 &= !pin.mask();
        self
    }

    /// Returns true if the specified pin is high.
    #[must_use]
    pub const fn is_high(self, pin: PinIndex) -> bool {
        self.0 & pin.mask() != 0
    }

    /// Returns true if the specified pin is low.
    #[must_use]
    pub const fn is_low(self, pin: PinIndex) -> bool {
        self.0 & pin.mask() == 0
    }
}

/// Polarity inversion registers.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[must_use]
pub struct PolarityInversion(pub u16);

impl PolarityInversion {
    /// Returns a new value with the specified pin inverted.
    pub const fn with_inverted(mut self, pin: PinIndex) -> Self {
        self.0 |= pin.mask();
        self
    }

    /// Returns a new value with the specified pin set to normal polarity.
    pub const fn with_normal(mut self, pin: PinIndex) -> Self {
        self.0 &= !pin.mask();
        self
    }

    /// Returns true if the specified pin is inverted.
    #[must_use]
    pub const fn is_inverted(self, pin: PinIndex) -> bool {
        self.0 & pin.mask() != 0
    }

    /// Returns true if the specified pin has normal polarity.
    #[must_use]
    pub const fn is_normal(self, pin: PinIndex) -> bool {
        self.0 & pin.mask() == 0
    }
}

/// Configuration registers.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[must_use]
pub struct Configuration(pub u16);

impl Configuration {
    /// Returns a new value with the specified pin configured as input.
    pub const fn with_input(mut self, pin: PinIndex) -> Self {
        self.0 |= pin.mask();
        self
    }

    /// Returns a new value with the specified pin configured as output.
    pub const fn with_output(mut self, pin: PinIndex) -> Self {
        self.0 &= !pin.mask();
        self
    }

    /// Returns true if the specified pin is configured as input.
    #[must_use]
    pub const fn is_input(self, pin: PinIndex) -> bool {
        self.0 & pin.mask() != 0
    }

    /// Returns true if the specified pin is configured as output.
    #[must_use]
    pub const fn is_output(self, pin: PinIndex) -> bool {
        self.0 & pin.mask() == 0
    }
}
