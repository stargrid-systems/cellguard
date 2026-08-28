//! The shared test registry and the outcome of a run.

/// Identifies one on-target test.
///
/// The wire name is the kebab-case id the host sends in `RUN` commands and
/// the firmware echoes in `result` lines. The numeric code is stable and
/// compact, for storage in the firmware's resume record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TestId {
    /// UART echo on the 4 MHz boot clock.
    UartEchoRc = 0,
    /// Switch the main clock to the 24 MHz external clock.
    ClockExtclk = 1,
    /// UART echo after the external-clock switch.
    UartEcho24m = 2,
    /// Non-destructive WEL round-trip on the app staging EEPROM (SPI0).
    Spi0Cat25ProbeApp = 3,
    /// Non-destructive WEL round-trip on the boot EEPROM (SPI0, CS PA7).
    Spi0Cat25ProbeBoot = 4,
    /// Non-destructive WEL round-trip on the factory identity EEPROM (SPI0,
    /// CS PG7).
    Spi0Cat25ProbeIdent = 5,
    /// Read the factory identity record from U106 and check magic and CRC.
    IdentRead = 6,
    /// Zero-length-write probe of every 7-bit address on TWI1.
    TwiScan = 7,
    /// Config-register write/readback on both TCA9535 expanders.
    Tca9535Readback = 8,
    /// Temperature read from the P3T1755 sensor.
    P3t1755Temp = 9,
}

impl TestId {
    /// Every known test, in the intended run order.
    pub const ALL: [Self; 10] = [
        Self::UartEchoRc,
        Self::ClockExtclk,
        Self::UartEcho24m,
        Self::Spi0Cat25ProbeApp,
        Self::Spi0Cat25ProbeBoot,
        Self::Spi0Cat25ProbeIdent,
        Self::IdentRead,
        Self::TwiScan,
        Self::Tca9535Readback,
        Self::P3t1755Temp,
    ];

    /// The kebab-case wire name.
    #[must_use]
    pub const fn name(self) -> &'static str {
        match self {
            Self::UartEchoRc => "uart-echo-rc",
            Self::ClockExtclk => "clock-extclk",
            Self::UartEcho24m => "uart-echo-24m",
            Self::Spi0Cat25ProbeApp => "spi0-cat25-probe-app",
            Self::Spi0Cat25ProbeBoot => "spi0-cat25-probe-boot",
            Self::Spi0Cat25ProbeIdent => "spi0-cat25-probe-ident",
            Self::IdentRead => "ident-read",
            Self::TwiScan => "twi-scan",
            Self::Tca9535Readback => "tca9535-readback",
            Self::P3t1755Temp => "p3t1755-temp",
        }
    }

    /// Looks up a test by its wire name.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|id| id.name() == name)
    }

    /// The stable numeric code.
    #[must_use]
    pub const fn code(self) -> u8 {
        self as u8
    }

    /// Looks up a test by its numeric code.
    #[must_use]
    pub fn from_code(code: u8) -> Option<Self> {
        Self::ALL.into_iter().find(|id| id.code() == code)
    }
}

/// Verdict of one test run.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Outcome {
    /// The test passed.
    Pass,
    /// The test failed.
    Fail,
    /// The test did not run, for the reason in the result detail.
    Skip,
}

impl Outcome {
    /// The wire token.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Pass => "PASS",
            Self::Fail => "FAIL",
            Self::Skip => "SKIP",
        }
    }

    /// Looks up an outcome by its wire token.
    #[must_use]
    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "PASS" => Some(Self::Pass),
            "FAIL" => Some(Self::Fail),
            "SKIP" => Some(Self::Skip),
            _ => None,
        }
    }
}
