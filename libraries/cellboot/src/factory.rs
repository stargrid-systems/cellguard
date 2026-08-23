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
use core::mem::size_of;

use zerocopy::byteorder::little_endian::{U16, U32};
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned};

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

/// Wire form of a [`FactoryRecord`]: the 64 bytes in the factory EEPROM.
#[derive(FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C)]
struct FactoryWire {
    magic: [u8; 4],
    format_version: u8,
    board_model: U16,
    board_revision: u8,
    reserved: [u8; 4],
    serial_cellcore: [u8; SERIAL_LEN],
    serial_cellagent: [u8; SERIAL_LEN],
    serial_cellprog: [u8; SERIAL_LEN],
    crc: U32,
}

const _: () = assert!(size_of::<FactoryWire>() == RECORD_LEN);

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
        let wire = FactoryWire {
            magic: MAGIC,
            format_version: FORMAT_VERSION,
            board_model: U16::new(self.board_model),
            board_revision: self.board_revision,
            reserved: [0; 4],
            serial_cellcore: self.serial_cellcore,
            serial_cellagent: self.serial_cellagent,
            serial_cellprog: self.serial_cellprog,
            crc: U32::new(0),
        };
        let mut out = [0u8; RECORD_LEN];
        out.copy_from_slice(wire.as_bytes());
        let crc = crc::checksum32(&out[..CRC_OFFSET]);
        out[CRC_OFFSET..].copy_from_slice(U32::new(crc).as_bytes());
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
        let wire = FactoryWire::ref_from_bytes(bytes).map_err(|_| ParseError::BadCrc)?;
        if crc::checksum32(&bytes[..CRC_OFFSET]) != wire.crc.get() {
            return Err(ParseError::BadCrc);
        }
        if wire.magic != MAGIC {
            return Err(ParseError::BadMagic);
        }
        if wire.format_version != FORMAT_VERSION {
            return Err(ParseError::UnsupportedFormat(wire.format_version));
        }
        Ok(Self {
            board_model: wire.board_model.get(),
            board_revision: wire.board_revision,
            serial_cellcore: wire.serial_cellcore,
            serial_cellagent: wire.serial_cellagent,
            serial_cellprog: wire.serial_cellprog,
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
    use super::{FORMAT_VERSION, FactoryRecord, MAGIC, ParseError, RECORD_LEN};

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
    fn wire_layout_is_frozen() {
        let bytes = sample().serialize();
        assert_eq!(&bytes[0..4], &MAGIC);
        assert_eq!(bytes[4], FORMAT_VERSION);
        assert_eq!(&bytes[5..7], &[0xEE, 0x0B]);
        assert_eq!(bytes[7], 7);
        assert_eq!(&bytes[8..12], &[0; 4]);
        let crc = crc::checksum32(&bytes[..60]);
        assert_eq!(&bytes[60..64], &crc.to_le_bytes());
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
