//! The factory identity record in the factory EEPROM (U106).
//!
//! U106 is a CAT25128 dedicated to factory settings and is never a
//! firmware-update target. The versioned record at offset 0 carries the
//! board identity and one serial number per node, in the same style as the
//! other storage records: fixed offsets, little-endian fields, and a CRC-32
//! over the record. Identity only: key material lives in the USERROW.
//!
//! Byte layout:
//!
//! - 0..4: magic `CGID`
//! - 4: format version
//! - 5..7: board model, little-endian
//! - 7: board revision
//! - 8..12: reserved, zero
//! - 12..28: cellcore serial
//! - 28..44: cellagent serial
//! - 44..60: cellprog serial
//! - 60..64: CRC-32 over bytes 0..60, little-endian

use core::fmt;

/// Factory record length in bytes.
pub const RECORD_LEN: usize = 64;

/// Magic bytes at the start of every factory record.
pub const MAGIC: [u8; 4] = *b"CGID";

/// Record format version understood by this crate.
pub const FORMAT_VERSION: u8 = 1;

/// Length of one node's serial number. Matches the AVR128 SIGROW serial, the
/// fallback identity of an unprovisioned board.
pub const SERIAL_LEN: usize = 16;

/// Offset of the record CRC. Every byte before it is covered by the CRC.
const CRC_OFFSET: usize = RECORD_LEN - 4;

/// The parsed factory record.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FactoryRecord {
    /// Board model. 0 is the unprovisioned marker on the bus (see the
    /// identity payload codecs of `cellguard-protocol`).
    pub board_model: u16,
    /// Board revision.
    pub board_revision: u8,
    /// Serial of the cellcore node (the AVR128).
    pub serial_cellcore: [u8; SERIAL_LEN],
    /// Serial of the cellagent node (the balancer `ATtiny406`).
    pub serial_cellagent: [u8; SERIAL_LEN],
    /// Serial of the cellprog node (the programmer `ATtiny406`).
    pub serial_cellprog: [u8; SERIAL_LEN],
}

impl FactoryRecord {
    /// Serializes the record into its canonical, CRC-protected byte form.
    #[must_use]
    pub fn serialize(&self) -> [u8; RECORD_LEN] {
        let mut out = [0u8; RECORD_LEN];
        out[0..4].copy_from_slice(&MAGIC);
        out[4] = FORMAT_VERSION;
        out[5..7].copy_from_slice(&self.board_model.to_le_bytes());
        out[7] = self.board_revision;
        out[12..28].copy_from_slice(&self.serial_cellcore);
        out[28..44].copy_from_slice(&self.serial_cellagent);
        out[44..60].copy_from_slice(&self.serial_cellprog);
        let crc = crc::checksum32(&out[0..CRC_OFFSET]);
        out[CRC_OFFSET..RECORD_LEN].copy_from_slice(&crc.to_le_bytes());
        out
    }

    /// Parses a record from its raw bytes.
    ///
    /// # Errors
    ///
    /// Returns a [`ParseError`] if the stored CRC does not match, the magic
    /// bytes are wrong, or the format version is not [`FORMAT_VERSION`]. A
    /// caller should treat an error as an unprovisioned board, not a reason
    /// to halt.
    pub fn parse(bytes: &[u8; RECORD_LEN]) -> Result<Self, ParseError> {
        let stored_crc = u32::from_le_bytes([
            bytes[CRC_OFFSET],
            bytes[CRC_OFFSET + 1],
            bytes[CRC_OFFSET + 2],
            bytes[CRC_OFFSET + 3],
        ]);
        if crc::checksum32(&bytes[0..CRC_OFFSET]) != stored_crc {
            return Err(ParseError::BadCrc);
        }
        if [bytes[0], bytes[1], bytes[2], bytes[3]] != MAGIC {
            return Err(ParseError::BadMagic);
        }
        if bytes[4] != FORMAT_VERSION {
            return Err(ParseError::UnsupportedFormat(bytes[4]));
        }

        let mut serial_cellcore = [0u8; SERIAL_LEN];
        serial_cellcore.copy_from_slice(&bytes[12..28]);
        let mut serial_cellagent = [0u8; SERIAL_LEN];
        serial_cellagent.copy_from_slice(&bytes[28..44]);
        let mut serial_cellprog = [0u8; SERIAL_LEN];
        serial_cellprog.copy_from_slice(&bytes[44..60]);

        Ok(Self {
            board_model: u16::from_le_bytes([bytes[5], bytes[6]]),
            board_revision: bytes[7],
            serial_cellcore,
            serial_cellagent,
            serial_cellprog,
        })
    }
}

/// An error returned when a factory record cannot be parsed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ParseError {
    /// The stored CRC did not match the contents.
    BadCrc,
    /// The magic bytes did not match [`MAGIC`].
    BadMagic,
    /// The record format version is not [`FORMAT_VERSION`].
    UnsupportedFormat(u8),
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadCrc => f.write_str("factory record CRC mismatch"),
            Self::BadMagic => f.write_str("bad factory record magic"),
            Self::UnsupportedFormat(v) => {
                write!(f, "unsupported factory record format version {v}")
            }
        }
    }
}

impl core::error::Error for ParseError {}

#[cfg(test)]
mod tests {
    use super::{FactoryRecord, MAGIC, ParseError, RECORD_LEN};

    fn sample() -> FactoryRecord {
        FactoryRecord {
            board_model: 0x0BEE,
            board_revision: 7,
            serial_cellcore: [1; 16],
            serial_cellagent: [2; 16],
            serial_cellprog: [3; 16],
        }
    }

    #[test]
    fn roundtrip() {
        let record = sample();
        assert_eq!(FactoryRecord::parse(&record.serialize()), Ok(record));
    }

    #[test]
    fn serials_live_at_distinct_offsets() {
        let bytes = sample().serialize();
        assert_eq!(bytes[12..28], [1; 16]);
        assert_eq!(bytes[28..44], [2; 16]);
        assert_eq!(bytes[44..60], [3; 16]);
    }

    #[test]
    fn blank_eeprom_fails_the_crc() {
        assert_eq!(
            FactoryRecord::parse(&[0xFF; RECORD_LEN]),
            Err(ParseError::BadCrc)
        );
    }

    #[test]
    fn detects_corruption() {
        let mut bytes = sample().serialize();
        bytes[20] ^= 0x01;
        assert_eq!(FactoryRecord::parse(&bytes), Err(ParseError::BadCrc));
    }

    #[test]
    fn detects_bad_magic() {
        let mut bytes = sample().serialize();
        bytes[0..4].copy_from_slice(b"XXXX");
        // Recompute the CRC so the magic check trips, not the CRC check.
        let crc = crc::checksum32(&bytes[0..60]);
        bytes[60..64].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(FactoryRecord::parse(&bytes), Err(ParseError::BadMagic));
    }

    #[test]
    fn detects_bad_format() {
        let mut bytes = sample().serialize();
        bytes[4] = 9;
        let crc = crc::checksum32(&bytes[0..60]);
        bytes[60..64].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(
            FactoryRecord::parse(&bytes),
            Err(ParseError::UnsupportedFormat(9))
        );
    }

    #[test]
    fn magic_is_stable() {
        assert_eq!(MAGIC, *b"CGID");
        assert_eq!(RECORD_LEN, 64);
    }
}
