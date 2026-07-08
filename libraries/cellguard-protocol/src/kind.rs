//! The central message-kind registry.
//!
//! Every message on the bus, request or response, has a [`Kind`]. Keeping them
//! in one enum makes the wire unambiguous and lets a trace tool decode any
//! packet. Discriminants are assigned as needed and must never be reused.
//! Bootloader kinds are compiled in only with the `bootloader` feature, but
//! keep fixed discriminants so a trace of a bootloader exchange reads the same
//! everywhere.

/// A message kind. The `u8` discriminant is what travels on the wire.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
#[repr(u8)]
pub enum Kind {
    /// Request: read the `ATtiny` 3-byte device id.
    ReadDeviceId = 1,
    /// Request: read the 10-byte serial number.
    ReadSerialNumber = 2,
    /// Request: read the temperature.
    ReadTemperature = 3,
    /// Response: device id.
    DeviceId = 4,
    /// Response: serial number.
    SerialNumber = 5,
    /// Response: temperature.
    Temperature = 6,
    /// Response: the request succeeded.
    Ack = 7,
    /// Response: the request was rejected.
    Nack = 8,

    /// Bootloader request: report update state.
    #[cfg(feature = "bootloader")]
    BootProbe = 9,
    /// Bootloader request: begin an update (payload is the image header).
    #[cfg(feature = "bootloader")]
    BootBegin = 10,
    /// Bootloader request: deliver a payload chunk.
    #[cfg(feature = "bootloader")]
    BootData = 11,
    /// Bootloader request: verify and stage the image.
    #[cfg(feature = "bootloader")]
    BootCommit = 12,
    /// Bootloader request: abandon the current update.
    #[cfg(feature = "bootloader")]
    BootAbort = 13,
    /// Bootloader response: the update state, in reply to `BootProbe`.
    #[cfg(feature = "bootloader")]
    BootStatus = 14,
    /// Bootloader response: a command succeeded.
    #[cfg(feature = "bootloader")]
    BootAck = 15,
    /// Bootloader response: a command was rejected.
    #[cfg(feature = "bootloader")]
    BootNack = 16,
}

impl Kind {
    /// Returns the wire byte for this kind.
    #[must_use]
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire byte into a kind, or `None` if it is not known.
    #[must_use]
    pub const fn from_u8(value: u8) -> Option<Self> {
        match value {
            1 => Some(Self::ReadDeviceId),
            2 => Some(Self::ReadSerialNumber),
            3 => Some(Self::ReadTemperature),
            4 => Some(Self::DeviceId),
            5 => Some(Self::SerialNumber),
            6 => Some(Self::Temperature),
            7 => Some(Self::Ack),
            8 => Some(Self::Nack),
            #[cfg(feature = "bootloader")]
            9 => Some(Self::BootProbe),
            #[cfg(feature = "bootloader")]
            10 => Some(Self::BootBegin),
            #[cfg(feature = "bootloader")]
            11 => Some(Self::BootData),
            #[cfg(feature = "bootloader")]
            12 => Some(Self::BootCommit),
            #[cfg(feature = "bootloader")]
            13 => Some(Self::BootAbort),
            #[cfg(feature = "bootloader")]
            14 => Some(Self::BootStatus),
            #[cfg(feature = "bootloader")]
            15 => Some(Self::BootAck),
            #[cfg(feature = "bootloader")]
            16 => Some(Self::BootNack),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Kind;

    #[test]
    fn roundtrips_known_kinds() {
        for byte in 0..=u8::MAX {
            if let Some(kind) = Kind::from_u8(byte) {
                assert_eq!(kind.to_u8(), byte);
            }
        }
    }

    #[test]
    fn rejects_zero() {
        assert_eq!(Kind::from_u8(0), None);
    }
}
