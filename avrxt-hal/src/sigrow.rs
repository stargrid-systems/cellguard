//! Signature row (SIGROW): read-only factory data.
//!
//! The signature row holds the device ID, a unique 16-byte serial number, and
//! the temperature sensor calibration. [`Sigrow`] is generic over a
//! [`SigrowInstance`] (implemented for each device's `SIGROW`).
//!
//! All values are programmed during production and never change. SIGROW is
//! read-only, so [`Sigrow`] borrows the peripheral instead of owning it and
//! every accessor takes `&self`.

/// The three device ID (signature) bytes, `SIGROW.DEVICEID[2:0]`.
///
/// For example an AVR128DA64 reads `[0x1E, 0x97, 0x07]`. The first byte is the
/// manufacturer (Microchip, `0x1E`), the last byte identifies the part.
pub type DeviceId = [u8; 3];

/// The 16-byte unique serial number, `SIGROW.SERNUM[15:0]`.
pub type SerialNumber = [u8; 16];

/// Temperature sensor calibration from the signature row.
///
/// `slope` is `SIGROW.TEMPSENSE0` and `offset` is `SIGROW.TEMPSENSE1`. Both are
/// determined in production for the internal 2.048 V reference. Use
/// [`kelvin`](Self::kelvin) or [`celsius`](Self::celsius) to turn an ADC
/// reading into a temperature.
#[derive(Clone, Copy)]
pub struct TempCalibration {
    slope: u16,
    offset: u16,
}

impl TempCalibration {
    /// Scaling factor the calibration values are stored against.
    const SCALING_FACTOR: u32 = 4096;

    /// The raw slope value (`SIGROW.TEMPSENSE0`).
    ///
    /// Exposed so the reading can be re-scaled for a reference other than the
    /// internal 2.048 V one (see the ADC section of the data sheet).
    #[must_use]
    pub const fn slope(self) -> u16 {
        self.slope
    }

    /// The raw offset value (`SIGROW.TEMPSENSE1`).
    #[must_use]
    pub const fn offset(self) -> u16 {
        self.offset
    }

    /// Converts a temperature sensor ADC reading into kelvin.
    ///
    /// `adc_reading` must be a 12-bit right-adjusted single-ended conversion of
    /// the temperature sensor taken with the internal 2.048 V reference. If
    /// samples are accumulated, scale the result back to 12 bits first.
    #[must_use]
    pub fn kelvin(self, adc_reading: u16) -> u16 {
        // T = (Offset - ADC * Slope) / 4096, with rounding. Matches the data
        // sheet reference code. Wrapping mirrors its unsigned 32-bit math.
        let mut temp = u32::from(self.offset).wrapping_sub(u32::from(adc_reading));
        temp = temp.wrapping_mul(u32::from(self.slope));
        temp = temp.wrapping_add(Self::SCALING_FACTOR / 2);
        (temp / Self::SCALING_FACTOR) as u16
    }

    /// Converts a temperature sensor ADC reading into degrees Celsius.
    ///
    /// See [`kelvin`](Self::kelvin) for the requirements on `adc_reading`.
    #[must_use]
    pub fn celsius(self, adc_reading: u16) -> i16 {
        self.kelvin(adc_reading) as i16 - 273
    }
}

/// A SIGROW peripheral. Implemented for each device's `SIGROW`. Not for
/// external use.
pub trait SigrowInstance {
    /// Reads the three device ID bytes.
    fn device_id(&self) -> DeviceId;
    /// Reads the gain/slope calibration word (`TEMPSENSE0`).
    fn temp_slope(&self) -> u16;
    /// Reads the offset calibration word (`TEMPSENSE1`).
    fn temp_offset(&self) -> u16;
    /// Reads the 16-byte serial number.
    fn serial_number(&self) -> SerialNumber;
}

/// The signature-row peripheral (borrowed, read-only).
pub struct Sigrow<'a, T: SigrowInstance> {
    instance: &'a T,
}

impl<'a, T: SigrowInstance> Sigrow<'a, T> {
    /// Borrows the SIGROW peripheral for reading.
    #[must_use]
    pub fn new(instance: &'a T) -> Self {
        Self { instance }
    }

    /// The three device ID (signature) bytes.
    #[must_use]
    pub fn device_id(&self) -> DeviceId {
        self.instance.device_id()
    }

    /// The unique 16-byte serial number.
    #[must_use]
    pub fn serial_number(&self) -> SerialNumber {
        self.instance.serial_number()
    }

    /// The temperature sensor calibration.
    #[must_use]
    pub fn temperature_calibration(&self) -> TempCalibration {
        TempCalibration {
            slope: self.instance.temp_slope(),
            offset: self.instance.temp_offset(),
        }
    }
}

macro_rules! impl_sigrow_instance {
    ($SIGROW:ty) => {
        impl SigrowInstance for $SIGROW {
            fn device_id(&self) -> DeviceId {
                [
                    self.deviceid0().read().bits(),
                    self.deviceid1().read().bits(),
                    self.deviceid2().read().bits(),
                ]
            }
            fn temp_slope(&self) -> u16 {
                self.tempsense0().read().bits()
            }
            fn temp_offset(&self) -> u16 {
                self.tempsense1().read().bits()
            }
            fn serial_number(&self) -> SerialNumber {
                [
                    self.sernum0().read().bits(),
                    self.sernum1().read().bits(),
                    self.sernum2().read().bits(),
                    self.sernum3().read().bits(),
                    self.sernum4().read().bits(),
                    self.sernum5().read().bits(),
                    self.sernum6().read().bits(),
                    self.sernum7().read().bits(),
                    self.sernum8().read().bits(),
                    self.sernum9().read().bits(),
                    self.sernum10().read().bits(),
                    self.sernum11().read().bits(),
                    self.sernum12().read().bits(),
                    self.sernum13().read().bits(),
                    self.sernum14().read().bits(),
                    self.sernum15().read().bits(),
                ]
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_sigrow_instance!(avr_device::avr128db48::SIGROW);
#[cfg(feature = "avr128db64")]
impl_sigrow_instance!(avr_device::avr128db64::SIGROW);
#[cfg(feature = "avr128da64")]
impl_sigrow_instance!(avr_device::avr128da64::SIGROW);
