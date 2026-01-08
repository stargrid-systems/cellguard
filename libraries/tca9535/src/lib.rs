#![no_std]

use embedded_hal::i2c::I2c;

pub use self::address::Address;

mod address;
mod pin;
mod register;

/// Driver for the TCA9535 16-bit I2C I/O expander.
///
/// Pins are exposed as a `u16` value, where each bit corresponds to a pin.
/// The least significant bit corresponds to pin P0, and the most significant
/// bit corresponds to pin P15.
/// The pins are split across two 8-bit ports: P0-P7 and P8-P15.
pub struct Tca9535<I> {
    i2c: I,
    addr: Address,
}

impl<I: I2c> Tca9535<I> {
    /// Creates a new driver instance.
    pub const fn new(i2c: I, addr: Address) -> Self {
        Self { i2c, addr }
    }

    pub fn read_input_ports(&mut self) -> Result<u16, I::Error> {
        self.read_register_pair(register::INPUT_PORT0)
    }

    pub fn read_output_ports(&mut self) -> Result<u16, I::Error> {
        self.read_register_pair(register::OUTPUT_PORT0)
    }

    pub fn write_output_ports(&mut self, value: u16) -> Result<(), I::Error> {
        self.write_register_pair(register::OUTPUT_PORT0, value)
    }

    pub fn read_polarity_inversion(&mut self) -> Result<u16, I::Error> {
        self.read_register_pair(register::POLARITY_INVERSION_PORT0)
    }

    pub fn write_polarity_inversion(&mut self, value: u16) -> Result<(), I::Error> {
        self.write_register_pair(register::POLARITY_INVERSION_PORT0, value)
    }

    pub fn read_configuration(&mut self) -> Result<u16, I::Error> {
        self.read_register_pair(register::CONFIGURATION_PORT0)
    }

    pub fn write_configuration(&mut self, value: u16) -> Result<(), I::Error> {
        self.write_register_pair(register::CONFIGURATION_PORT0, value)
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
