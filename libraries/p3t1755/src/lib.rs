//! NXP P3T1755 temperature sensor driver.

#![no_std]

use embedded_hal::i2c::{I2c, Operation};

pub use self::address::Address;
use self::register::Register;
pub use self::register::{Config, ConversionTime, FaultQueue, Temperature};

mod address;
pub mod alert;
mod register;

/// P3T1755 temperature sensor driver.
///
/// Generic I2C driver for the P3T1755 digital temperature sensor.
pub struct P3t1755<I> {
    i2c: I,
    addr: Address,
    latched_reg: Option<Register>,
}

impl<I: I2c> P3t1755<I> {
    /// Creates a new driver instance with the given I2C interface and address.
    pub const fn new(i2c: I, addr: Address) -> Self {
        Self {
            addr,
            i2c,
            latched_reg: None,
        }
    }

    /// Consumes the driver and returns the underlying I2C interface.
    pub fn into_inner(self) -> I {
        self.i2c
    }

    /// Reads the configuration register.
    pub fn read_config(&mut self) -> Result<Config, I::Error> {
        let mut buf = [0u8; 1];
        self.read_register(Register::Conf, &mut buf)?;
        Ok(Config::from_reg(buf[0]))
    }

    /// Writes the configuration register.
    pub fn write_config(&mut self, config: Config) -> Result<(), I::Error> {
        self.write_register(Register::Conf, &[config.to_reg()])
    }

    /// Reads the `TLOW` register.
    pub fn read_t_low(&mut self) -> Result<Temperature, I::Error> {
        let mut buf = [0u8; 2];
        self.read_register(Register::TLow, &mut buf)?;
        Ok(Temperature::from_regs(&buf))
    }

    /// Writes the `TLOW` register.
    pub fn write_t_low(&mut self, temp: Temperature) -> Result<(), I::Error> {
        self.write_register(Register::TLow, &temp.to_regs())
    }

    /// Reads the `THIGH` register.
    pub fn read_t_high(&mut self) -> Result<Temperature, I::Error> {
        let mut buf = [0u8; 2];
        self.read_register(Register::THigh, &mut buf)?;
        Ok(Temperature::from_regs(&buf))
    }

    /// Writes the `THIGH` register.
    pub fn write_t_high(&mut self, temp: Temperature) -> Result<(), I::Error> {
        self.write_register(Register::THigh, &temp.to_regs())
    }

    /// Reads the temperature register.
    pub fn read_temperature(&mut self) -> Result<Temperature, I::Error> {
        let mut buf = [0u8; 2];
        self.read_register(Register::Temp, &mut buf)?;
        Ok(Temperature::from_regs(&buf))
    }

    fn read_register(&mut self, reg: Register, buf: &mut [u8]) -> Result<(), I::Error> {
        let operations: &mut [Operation<'_>] = if self.latched_reg == Some(reg) {
            // We can skip writing to the pointer because it's already set.
            &mut [Operation::Read(buf)]
        } else {
            &mut [Operation::Write(&[reg.get()]), Operation::Read(buf)]
        };
        self.i2c.transaction(self.addr.get(), operations)?;
        self.latched_reg = Some(reg);
        Ok(())
    }

    fn write_register(&mut self, reg: Register, buf: &[u8]) -> Result<(), I::Error> {
        self.i2c.transaction(
            self.addr.get(),
            &mut [Operation::Write(&[reg.get()]), Operation::Write(buf)],
        )?;
        self.latched_reg = Some(reg);
        Ok(())
    }
}
