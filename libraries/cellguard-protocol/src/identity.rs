//! Payload codecs for the device-identity kinds.
//!
//! [`Kind::ReadDeviceId`](crate::Kind::ReadDeviceId) asks a node for its
//! board identity and firmware version,
//! [`Kind::ReadSerialNumber`](crate::Kind::ReadSerialNumber) for its serial
//! number. Payloads are little-endian throughout, like every other frame.

/// Length of the serial number in a
/// [`Kind::SerialNumber`](crate::Kind::SerialNumber) payload. Matches the
/// AVR128 SIGROW serial and the factory record.
pub const SERIAL_LEN: usize = 16;

/// Board model marking an unprovisioned board: the node has no factory
/// record and reports its chip identity.
pub const BOARD_MODEL_UNPROVISIONED: u16 = 0;

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
    pub const PAYLOAD_LEN: usize = 7;

    /// Encodes into `out`, returning the payload slice.
    #[must_use]
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> Option<&'a [u8]> {
        let buf = out.get_mut(..Self::PAYLOAD_LEN)?;
        let (model, rest) = buf.split_at_mut(2);
        model.copy_from_slice(&self.board_model.to_le_bytes());
        let (revision, rest) = rest.split_first_mut()?;
        *revision = self.board_revision;
        rest.copy_from_slice(&self.fw_version.to_le_bytes());
        out.get(..Self::PAYLOAD_LEN)
    }

    /// Decodes a payload into a device id.
    #[must_use]
    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != Self::PAYLOAD_LEN {
            return None;
        }
        let (model, rest) = payload.split_first_chunk::<2>()?;
        let (revision, rest) = rest.split_first_chunk::<1>()?;
        let (fw_version, _) = rest.split_first_chunk::<4>()?;
        Some(Self {
            board_model: u16::from_le_bytes(*model),
            board_revision: revision[0],
            fw_version: u32::from_le_bytes(*fw_version),
        })
    }
}

/// Encodes `serial` into `out`, returning the payload slice for a
/// [`Kind::SerialNumber`](crate::Kind::SerialNumber) reply.
#[must_use]
pub fn encode_serial<'a>(serial: &[u8; SERIAL_LEN], out: &'a mut [u8]) -> Option<&'a [u8]> {
    out.get_mut(..SERIAL_LEN)?.copy_from_slice(serial);
    out.get(..SERIAL_LEN)
}

/// Decodes a [`Kind::SerialNumber`](crate::Kind::SerialNumber) payload into
/// the node's serial number.
#[must_use]
pub fn decode_serial(payload: &[u8]) -> Option<[u8; SERIAL_LEN]> {
    let (serial, rest) = payload.split_first_chunk::<SERIAL_LEN>()?;
    if !rest.is_empty() {
        return None;
    }
    Some(*serial)
}

#[cfg(test)]
mod tests {
    use super::{BOARD_MODEL_UNPROVISIONED, DeviceId, SERIAL_LEN, decode_serial, encode_serial};

    #[test]
    fn device_id_roundtrips() {
        let id = DeviceId {
            board_model: 0x1234,
            board_revision: 0x56,
            fw_version: 0x0BAD_C0DE,
        };
        let mut buf = [0u8; DeviceId::PAYLOAD_LEN + 3];
        let payload = id.encode(&mut buf).expect("fits");
        assert_eq!(payload.len(), DeviceId::PAYLOAD_LEN);
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
        let serial = [0xAB; SERIAL_LEN];
        let mut buf = [0u8; SERIAL_LEN + 1];
        let payload = encode_serial(&serial, &mut buf).expect("fits");
        assert_eq!(decode_serial(payload), Some(serial));

        assert!(decode_serial(&[0; SERIAL_LEN - 1]).is_none());
        assert!(decode_serial(&[0; SERIAL_LEN + 1]).is_none());
        assert!(encode_serial(&serial, &mut [0u8; SERIAL_LEN - 1]).is_none());
    }

    #[test]
    fn unprovisioned_model_is_zero() {
        assert_eq!(BOARD_MODEL_UNPROVISIONED, 0);
    }
}
