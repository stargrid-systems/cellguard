//! The central message-kind registry.
//!
//! Every message on the bus, request or response, has a [`Kind`]. Keeping them
//! in one enum makes the wire unambiguous and lets a trace tool decode any
//! packet. Discriminants are assigned as needed and must never be reused.
//! Bootloader kinds are compiled in only with the `bootloader` feature, but
//! keep fixed discriminants so a trace of a bootloader exchange reads the same
//! everywhere.

use zerocopy::{Immutable, IntoBytes, KnownLayout, TryFromBytes, Unaligned};

/// A message kind. The `u8` discriminant is what travels on the wire.
///
/// Not every byte is a valid kind, so this derives [`TryFromBytes`] rather than
/// `FromBytes`: an unknown discriminant is rejected instead of accepted.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, TryFromBytes, IntoBytes, Immutable, KnownLayout, Unaligned,
)]
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
    /// Bootloader request: replace the shared authentication key. The payload
    /// is the new key followed by an authentication tag over it.
    /// Development use only, and inert once the key store is locked.
    #[cfg(feature = "bootloader")]
    BootReplaceKey = 17,
    /// Programmer request (main MCU to `cellprog`): program a staged image into
    /// a target. The payload selects which staged image.
    #[cfg(feature = "bootloader")]
    ProgProgram = 18,
    /// Programmer result (`cellprog` to main MCU): the outcome of a
    /// [`Kind::ProgProgram`] request.
    #[cfg(feature = "bootloader")]
    ProgResult = 19,
    /// Request: set the cellagent balancer gate state.
    /// Payload: 1 byte bitmask (bit 0 = `GATE_A`, bit 1 = `GATE_B`, bit 2 = `ALL_OFF`).
    SetBalancer = 20,
    /// Request: report the last panic record. Available on every node.
    PanicProbe = 21,
    /// Response: the panic record, in reply to [`Kind::PanicProbe`]. An empty
    /// payload means no panic is recorded for this session.
    PanicStatus = 22,
}

impl Kind {
    /// Returns the wire byte for this kind.
    #[must_use]
    pub const fn to_u8(self) -> u8 {
        self as u8
    }

    /// Parses a wire byte into a kind, or `None` if it is not known.
    ///
    /// Validity is derived from the enum discriminants by [`TryFromBytes`], so
    /// this cannot drift out of sync with the variants.
    #[must_use]
    pub fn from_u8(value: u8) -> Option<Self> {
        Self::try_read_from_bytes(&[value]).ok()
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
