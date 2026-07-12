//! The programmer messages exchanged over the local `UART_PROG` link.
//!
//! A [`Kind::ProgProgram`](crate::Kind::ProgProgram) request carries a single
//! [`ProgSource`] byte selecting which staged image to flash. The programmer
//! reads that image straight from the shared EEPROM, so the image bytes never
//! cross the link. A [`Kind::ProgResult`](crate::Kind::ProgResult) reply
//! carries a single [`ProgStatus`] byte. Both roles share these definitions:
//! the update agent sends the request, the `cellprog` programmer answers it.

/// Which staged image the programmer should flash.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProgSource {
    /// The application image staged in the App Code region.
    AppStaged,
    /// The bootloader image staged in the Bootloader region.
    BootloaderStaged,
    /// The known-good golden image, used for recovery.
    Golden,
}

impl ProgSource {
    /// Returns the wire byte for this source.
    #[must_use]
    pub const fn to_code(self) -> u8 {
        match self {
            Self::AppStaged => 0,
            Self::BootloaderStaged => 1,
            Self::Golden => 2,
        }
    }

    /// Parses a wire byte into a source.
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::AppStaged),
            1 => Some(Self::BootloaderStaged),
            2 => Some(Self::Golden),
            _ => None,
        }
    }
}

/// The outcome of a program attempt, reported in a `ProgResult`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProgStatus {
    /// The target was programmed, verified, and released.
    Ok,
    /// The staged image did not match its CRC.
    CorruptSource,
    /// The written flash did not match its CRC.
    VerifyFailed,
    /// The store or writer failed, or the header did not parse.
    Failed,
}

impl ProgStatus {
    /// Returns the wire byte for this status.
    #[must_use]
    pub const fn to_code(self) -> u8 {
        match self {
            Self::Ok => 0,
            Self::CorruptSource => 1,
            Self::VerifyFailed => 2,
            Self::Failed => 3,
        }
    }

    /// Parses a wire byte into a status.
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Ok),
            1 => Some(Self::CorruptSource),
            2 => Some(Self::VerifyFailed),
            3 => Some(Self::Failed),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ProgSource, ProgStatus};

    #[test]
    fn source_roundtrips() {
        for source in [
            ProgSource::AppStaged,
            ProgSource::BootloaderStaged,
            ProgSource::Golden,
        ] {
            assert_eq!(ProgSource::from_code(source.to_code()), Some(source));
        }
        assert_eq!(ProgSource::from_code(3), None);
    }

    #[test]
    fn status_roundtrips() {
        for status in [
            ProgStatus::Ok,
            ProgStatus::CorruptSource,
            ProgStatus::VerifyFailed,
            ProgStatus::Failed,
        ] {
            assert_eq!(ProgStatus::from_code(status.to_code()), Some(status));
        }
        assert_eq!(ProgStatus::from_code(4), None);
    }
}
