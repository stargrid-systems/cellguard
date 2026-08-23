//! Device identity for the core node.
//!
//! [`Identity`] holds the board identity and serial this node serves for the
//! identity request kinds. The firmware builds it once at boot: from the
//! factory EEPROM record (U106) when the record parses, otherwise from the
//! chip's SIGROW serial and the unprovisioned board model. A bad record
//! never blocks boot.

use cellboot::factory::{FactoryRecord, SERIAL_LEN as FACTORY_SERIAL_LEN};
use cellguard_protocol::{BOARD_MODEL_UNPROVISIONED, DeviceId, Kind, SERIAL_LEN, SerialNumber};

// The factory record and the wire payload must agree on the serial width.
const _: () = assert!(FACTORY_SERIAL_LEN == SERIAL_LEN);

/// The identity this node reports for the identity request kinds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Identity {
    id: DeviceId,
    serial: SerialNumber,
}

impl Identity {
    /// Builds the identity from `record` when it parsed, falling back to the
    /// chip serial and the unprovisioned board model otherwise.
    ///
    /// `fw_version` is the running firmware's version, reported in the
    /// [`Kind::DeviceId`] reply.
    #[must_use]
    pub const fn from_factory_record(
        record: Option<FactoryRecord>,
        chip_serial: [u8; SERIAL_LEN],
        fw_version: u32,
    ) -> Self {
        let mut this = Self {
            id: DeviceId {
                board_model: BOARD_MODEL_UNPROVISIONED,
                board_revision: 0,
                fw_version,
            },
            serial: SerialNumber {
                serial: chip_serial,
            },
        };
        if let Some(record) = record {
            this.id.board_model = record.board_model;
            this.id.board_revision = record.board_revision;
            this.serial = SerialNumber {
                serial: record.serial_cellcore,
            };
        }
        this
    }

    /// Serves one identity request into `out`. Returns `None` for kinds this
    /// layer does not own, so another layer may take them.
    #[must_use]
    pub fn handle(&self, kind: Kind, out: &mut [u8]) -> Option<(Kind, usize)> {
        match kind {
            Kind::ReadDeviceId => {
                let payload = self.id.encode(out)?;
                Some((Kind::DeviceId, payload.len()))
            }
            Kind::ReadSerialNumber => {
                let payload = self.serial.encode(out)?;
                Some((Kind::SerialNumber, payload.len()))
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use cellboot::factory::FactoryRecord;
    use cellguard_protocol::{DeviceId, Kind, SerialNumber};

    use super::Identity;

    const CHIP_SERIAL: [u8; 16] = [0xA5; 16];
    const FW_VERSION: u32 = 0x0102_0304;

    fn record() -> FactoryRecord {
        FactoryRecord {
            board_model: 9,
            board_revision: 3,
            serial_cellcore: [0x11; 16],
            serial_cellagent: [0x22; 16],
            serial_cellprog: [0x33; 16],
        }
    }

    fn reply(identity: &Identity, kind: Kind) -> Option<(Kind, [u8; 16], usize)> {
        let mut out = [0u8; 16];
        identity
            .handle(kind, &mut out)
            .map(|(kind, len)| (kind, out, len))
    }

    #[test]
    fn factory_record_supplies_board_and_serial() {
        let identity = Identity::from_factory_record(Some(record()), CHIP_SERIAL, FW_VERSION);
        let (kind, out, len) = reply(&identity, Kind::ReadDeviceId).unwrap();
        assert_eq!(kind, Kind::DeviceId);
        assert_eq!(len, DeviceId::PAYLOAD_LEN);
        let id = DeviceId::decode(&out[..len]).unwrap();
        assert_eq!(id.board_model, 9);
        assert_eq!(id.board_revision, 3);
        assert_eq!(id.fw_version, FW_VERSION);

        let (kind, out, len) = reply(&identity, Kind::ReadSerialNumber).unwrap();
        assert_eq!(kind, Kind::SerialNumber);
        assert_eq!(
            SerialNumber::decode(&out[..len]),
            Some(SerialNumber { serial: [0x11; 16] })
        );
    }

    #[test]
    fn missing_record_falls_back_to_chip_serial_and_unprovisioned_model() {
        let identity = Identity::from_factory_record(None, CHIP_SERIAL, FW_VERSION);
        let (kind, out, len) = reply(&identity, Kind::ReadDeviceId).unwrap();
        assert_eq!(kind, Kind::DeviceId);
        let id = DeviceId::decode(&out[..len]).unwrap();
        assert_eq!(
            id.board_model,
            cellguard_protocol::BOARD_MODEL_UNPROVISIONED
        );
        assert_eq!(id.board_revision, 0);
        assert_eq!(id.fw_version, FW_VERSION);

        let (kind, out, len) = reply(&identity, Kind::ReadSerialNumber).unwrap();
        assert_eq!(kind, Kind::SerialNumber);
        assert_eq!(
            SerialNumber::decode(&out[..len]),
            Some(SerialNumber {
                serial: CHIP_SERIAL
            })
        );
    }

    #[test]
    fn other_kinds_are_not_owned() {
        let identity = Identity::from_factory_record(Some(record()), CHIP_SERIAL, FW_VERSION);
        let mut out = [0u8; 16];
        assert!(identity.handle(Kind::ReadTemperature, &mut out).is_none());
        assert!(identity.handle(Kind::BootProbe, &mut out).is_none());
    }
}
