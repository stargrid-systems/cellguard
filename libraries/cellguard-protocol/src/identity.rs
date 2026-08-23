//! Payload codecs for the device-identity kinds.
//!
//! [`Kind::ReadDeviceId`](crate::Kind::ReadDeviceId) asks a node for its
//! board identity and firmware version,
//! [`Kind::ReadSerialNumber`](crate::Kind::ReadSerialNumber) for its serial
//! number. Payloads are little-endian throughout, like every other frame.

use core::mem::size_of;

use zerocopy::byteorder::little_endian::{U16, U32};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned};

/// Length of the serial number in a
/// [`Kind::SerialNumber`](crate::Kind::SerialNumber) payload. Matches the
/// AVR128 SIGROW serial and the factory record.
pub const SERIAL_LEN: usize = 16;

/// Board model marking an unprovisioned board: the node has no factory
/// record and reports its chip identity.
pub const BOARD_MODEL_UNPROVISIONED: u16 = 0;

/// Wire form of a [`DeviceId`] payload.
#[derive(FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C)]
struct DeviceIdWire {
    board_model: U16,
    board_revision: u8,
    fw_version: U32,
}

/// Decoded [`Kind::DeviceId`](crate::Kind::DeviceId) payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DeviceId {
    /// Board model from the factory record, or
    /// [`BOARD_MODEL_UNPROVISIONED`] when the board has none.
    pub board_model: u16,
    /// Board revision from the factory record.
    pub board_revision: u8,
    /// Version of the running firmware.
    pub fw_version: u32,
}

impl DeviceId {
    /// Payload length of the encoded form.
    pub const PAYLOAD_LEN: usize = size_of::<DeviceIdWire>();

    /// Encodes into `out`, returning the payload slice.
    #[must_use]
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> Option<&'a [u8]> {
        let wire = DeviceIdWire {
            board_model: U16::new(self.board_model),
            board_revision: self.board_revision,
            fw_version: U32::new(self.fw_version),
        };
        out.get_mut(..Self::PAYLOAD_LEN)?
            .copy_from_slice(wire.as_bytes());
        out.get(..Self::PAYLOAD_LEN)
    }

    /// Decodes a payload into a device id.
    #[must_use]
    pub fn decode(payload: &[u8]) -> Option<Self> {
        let wire = DeviceIdWire::ref_from_bytes(payload).ok()?;
        Some(Self {
            board_model: wire.board_model.get(),
            board_revision: wire.board_revision,
            fw_version: wire.fw_version.get(),
        })
    }
}

/// Wire form of a [`SerialNumber`] payload.
#[derive(FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C)]
struct SerialNumberWire {
    serial: [u8; SERIAL_LEN],
}

/// Decoded [`Kind::SerialNumber`](crate::Kind::SerialNumber) payload: the
/// node's serial number.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SerialNumber {
    /// The serial bytes.
    pub serial: [u8; SERIAL_LEN],
}

impl SerialNumber {
    /// Payload length of the encoded form.
    pub const PAYLOAD_LEN: usize = size_of::<SerialNumberWire>();

    /// Encodes into `out`, returning the payload slice.
    #[must_use]
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> Option<&'a [u8]> {
        let wire = SerialNumberWire {
            serial: self.serial,
        };
        out.get_mut(..Self::PAYLOAD_LEN)?
            .copy_from_slice(wire.as_bytes());
        out.get(..Self::PAYLOAD_LEN)
    }

    /// Decodes a payload into a serial number.
    #[must_use]
    pub fn decode(payload: &[u8]) -> Option<Self> {
        let wire = SerialNumberWire::ref_from_bytes(payload).ok()?;
        Some(Self {
            serial: wire.serial,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::{BOARD_MODEL_UNPROVISIONED, DeviceId, SERIAL_LEN, SerialNumber};

    #[test]
    fn device_id_wire_bytes_are_frozen() {
        let id = DeviceId {
            board_model: 0x1234,
            board_revision: 0x56,
            fw_version: 0x0BAD_C0DE,
        };
        let mut buf = [0u8; DeviceId::PAYLOAD_LEN + 3];
        let payload = id.encode(&mut buf).expect("fits");
        assert_eq!(
            payload,
            &[0x34, 0x12, 0x56, 0xDE, 0xC0, 0xAD, 0x0B],
            "model LE, revision, fw version LE"
        );
        assert_eq!(DeviceId::decode(payload), Some(id));
    }

    #[test]
    fn device_id_rejects_wrong_length() {
        assert!(DeviceId::decode(&[0; 6]).is_none());
        assert!(DeviceId::decode(&[0; 8]).is_none());
        assert!(
            DeviceId {
                board_model: 0,
                board_revision: 0,
                fw_version: 0
            }
            .encode(&mut [0u8; 4])
            .is_none()
        );
    }

    #[test]
    fn serial_roundtrips_and_rejects_wrong_length() {
        let serial = SerialNumber {
            serial: [0xAB; SERIAL_LEN],
        };
        let mut buf = [0u8; SERIAL_LEN + 1];
        let payload = serial.encode(&mut buf).expect("fits");
        assert_eq!(payload, &[0xAB; SERIAL_LEN]);
        assert_eq!(SerialNumber::decode(payload), Some(serial));

        assert!(SerialNumber::decode(&[0; SERIAL_LEN - 1]).is_none());
        assert!(SerialNumber::decode(&[0; SERIAL_LEN + 1]).is_none());
        assert!(serial.encode(&mut [0u8; SERIAL_LEN - 1]).is_none());
    }

    #[test]
    fn unprovisioned_model_is_zero() {
        assert_eq!(BOARD_MODEL_UNPROVISIONED, 0);
    }
}
