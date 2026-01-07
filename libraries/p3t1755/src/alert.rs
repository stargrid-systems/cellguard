//! Alert handling for the P3T1755 temperature sensor.
//!
//! The alert handling is based on the "SMBus Alert Response".

use embedded_hal::i2c::{Error, ErrorKind, I2c, NoAcknowledgeSource};

use crate::Address;

/// Alert information returned by the P3T1755 sensor.
#[derive(Clone, Copy)]
pub struct Alert(u8);

impl Alert {
    /// Creates a new Alert from a raw byte.
    ///
    /// # Safety
    ///
    /// The caller must ensure that the provided byte corresponds to a valid
    /// alert condition and address combination.
    const unsafe fn new(byte: u8) -> Self {
        Self(byte)
    }

    const fn from_byte(byte: u8) -> Option<Self> {
        let address_bits = byte >> 1;
        // Validate that the address bits correspond to a valid Address enum variant.
        if Address::new(address_bits).is_some() {
            // SAFETY: We have validated the address bits.
            Some(unsafe { Self::new(byte) })
        } else {
            None
        }
    }

    /// Returns the I2C address of the device that triggered the alert.
    pub const fn address(self) -> Address {
        let bits = self.0 >> 1;
        // SAFETY: We only allow alerts containing valid address bits to be created.
        unsafe { Address::new(bits).unwrap_unchecked() }
    }

    /// Returns the alert condition.
    pub const fn condition(self) -> AlertCondition {
        // If the temperature is higher than THIGH, the LSB bit is 1.
        // If the temperature is lower than TLOW, the LSB bit is 0.
        if (self.0 & 0x01) == 0 {
            AlertCondition::UnderTemperature
        } else {
            AlertCondition::OverTemperature
        }
    }
}

/// Alert condition.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum AlertCondition {
    /// Temperature exceeded THIGH threshold.
    OverTemperature,
    /// Temperature fell below TLOW threshold.
    UnderTemperature,
}

/// Processes an alert on the given I2C bus.
///
/// Returns `None` if no device acknowledged the alert request (i.e. no alert is
/// pending) or if the response doesn't correspond to a valid P3T1755 device
/// address.
pub fn process<I: I2c>(bus: &mut I) -> Result<Option<Alert>, I::Error> {
    let mut buf = [0u8; 1];
    if let Err(err) = bus.read(0x0C, &mut buf) {
        if matches!(
            err.kind(),
            ErrorKind::NoAcknowledge(NoAcknowledgeSource::Address)
        ) {
            // No alert pending.
            return Ok(None);
        }
        // Some other error occurred.
        return Err(err);
    }
    Ok(Alert::from_byte(buf[0]))
}
