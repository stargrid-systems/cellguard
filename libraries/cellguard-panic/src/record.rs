//! The persistent panic record.
//!
//! [`PanicRecord`] is a fixed-size, versioned, CRC-32-protected record that a
//! panic handler stores in on-chip EEPROM so a later boot or a field-bus probe
//! can read back why and where the device panicked. It captures only the panic
//! location (file, line, column), the reset-cause flags active at the time, and
//! a crash-loop counter. The payload is intentionally compact to fit the
//! 128-byte EEPROM of the tinyAVR co-processors as well as the AVR128 core.
//!
//! The layout mirrors `cellcore::update::state::PersistentState`: a format
//! version byte, the fields, and a trailing CRC-32 over everything before it.
//! Parsing falls back gracefully: a blank EEPROM cell (`0xFF`) or a corrupt
//! record fails the version or CRC check, which a caller treats as "no record".

use core::panic::PanicInfo;

use crate::ParseError;

/// On-wire record format version understood by this crate.
pub const RECORD_FORMAT_VERSION: u8 = 1;

/// Maximum number of source-file path bytes kept.
pub const FILE_CAP: usize = 48;

/// Total serialized length of a [`PanicRecord`] in bytes.
pub const RECORD_LEN: usize = 64;

const FILE_OFF: usize = 4;
const LINE_OFF: usize = FILE_OFF + FILE_CAP;
const COL_OFF: usize = LINE_OFF + 4;
const CRC_OFF: usize = COL_OFF + 4;

const _: () = assert!(CRC_OFF + 4 == RECORD_LEN);

/// The persistent panic record.
///
/// Build it from a [`PanicInfo`] with [`PanicRecord::from_panic_info`], then
/// [`PanicRecord::serialize`] it for storage. Recover it with
/// [`PanicRecord::parse`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PanicRecord {
    /// Reset-cause flags (`RSTCTRL.RSTFR` bits) active when the panic fired.
    pub reset_flags: u8,
    /// Consecutive panic-resets at the time this record was written. A healthy
    /// boot clears it back to zero.
    pub consecutive_panics: u8,
    /// Source-file path bytes, truncated to [`FILE_CAP`].
    pub file: [u8; FILE_CAP],
    /// Number of valid bytes in [`file`](Self::file).
    pub file_len: u8,
    /// `file!()` line number, or `0` if the location was unavailable.
    pub line: u32,
    /// `column!()` column number, or `0` if the location was unavailable.
    pub col: u32,
}

impl PanicRecord {
    /// Builds a record from a panic, capturing its location and the given reset
    /// flags. The crash-loop counter starts at zero; the storage layer sets it.
    #[must_use]
    pub fn from_panic_info(info: &PanicInfo, reset_flags: u8) -> Self {
        let (file, file_len, line, col) = location_bytes(info);
        Self {
            reset_flags,
            consecutive_panics: 0,
            file,
            file_len,
            line,
            col,
        }
    }

    /// Serializes the record into its canonical, CRC-protected byte form.
    #[must_use]
    pub fn serialize(&self) -> [u8; RECORD_LEN] {
        let mut out = [0u8; RECORD_LEN];
        out[0] = RECORD_FORMAT_VERSION;
        out[1] = self.reset_flags;
        out[2] = self.consecutive_panics;
        out[3] = self.file_len;
        out[FILE_OFF..FILE_OFF + FILE_CAP].copy_from_slice(&self.file);
        out[LINE_OFF..LINE_OFF + 4].copy_from_slice(&self.line.to_le_bytes());
        out[COL_OFF..COL_OFF + 4].copy_from_slice(&self.col.to_le_bytes());
        let crc = crc::checksum32(&out[0..CRC_OFF]);
        out[CRC_OFF..CRC_OFF + 4].copy_from_slice(&crc.to_le_bytes());
        out
    }

    /// Parses a record from its bytes.
    ///
    /// # Errors
    ///
    /// [`ParseError::BadCrc`] if the CRC does not match,
    /// [`ParseError::UnsupportedVersion`] if the format version is not
    /// [`RECORD_FORMAT_VERSION`] (a blank `0xFF` cell lands here), or
    /// [`ParseError::BadField`] if the stored file length is out of range.
    pub fn parse(bytes: &[u8; RECORD_LEN]) -> Result<Self, ParseError> {
        let stored = u32::from_le_bytes([
            bytes[CRC_OFF],
            bytes[CRC_OFF + 1],
            bytes[CRC_OFF + 2],
            bytes[CRC_OFF + 3],
        ]);
        if crc::checksum32(&bytes[0..CRC_OFF]) != stored {
            return Err(ParseError::BadCrc);
        }
        if bytes[0] != RECORD_FORMAT_VERSION {
            return Err(ParseError::UnsupportedVersion(bytes[0]));
        }
        let file_len = bytes[3];
        if usize::from(file_len) > FILE_CAP {
            return Err(ParseError::BadField);
        }
        let mut file = [0u8; FILE_CAP];
        file.copy_from_slice(&bytes[FILE_OFF..FILE_OFF + FILE_CAP]);
        let line = u32::from_le_bytes([
            bytes[LINE_OFF],
            bytes[LINE_OFF + 1],
            bytes[LINE_OFF + 2],
            bytes[LINE_OFF + 3],
        ]);
        let col = u32::from_le_bytes([
            bytes[COL_OFF],
            bytes[COL_OFF + 1],
            bytes[COL_OFF + 2],
            bytes[COL_OFF + 3],
        ]);
        Ok(Self {
            reset_flags: bytes[1],
            consecutive_panics: bytes[2],
            file,
            file_len,
            line,
            col,
        })
    }

    /// Returns the stored source-file path as a `&str`, or `None` if the bytes
    /// are not valid UTF-8 or no path was recorded.
    #[must_use]
    pub fn file_str(&self) -> Option<&str> {
        let bytes = self.file.get(..usize::from(self.file_len))?;
        core::str::from_utf8(bytes).ok()
    }
}

/// Extracts the panic location into a fixed-size buffer plus line/column.
fn location_bytes(info: &PanicInfo) -> ([u8; FILE_CAP], u8, u32, u32) {
    let Some(loc) = info.location() else {
        return ([0u8; FILE_CAP], 0, 0, 0);
    };
    let mut file = [0u8; FILE_CAP];
    let file_len = store_path(loc.file().as_bytes(), &mut file);
    (file, file_len, loc.line(), loc.column())
}

/// Copies up to [`FILE_CAP`] bytes of `path` into `buf`, returning the count.
/// Paths longer than the cap keep their leading bytes, which carry the
/// crate/module and are usually enough to locate the panic.
fn store_path(path: &[u8], buf: &mut [u8; FILE_CAP]) -> u8 {
    let n = path.len().min(FILE_CAP);
    let (dst, _) = buf.split_at_mut(n);
    let (src, _) = path.split_at(n);
    dst.copy_from_slice(src);
    // `n` is clamped to `FILE_CAP` (48), so it always fits in a `u8`.
    u8::try_from(n).unwrap_or(u8::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> PanicRecord {
        let mut file = [0u8; FILE_CAP];
        let path = b"src/main.rs";
        file[..path.len()].copy_from_slice(path);
        PanicRecord {
            reset_flags: 0x14,
            consecutive_panics: 2,
            file,
            file_len: u8::try_from(path.len()).unwrap(),
            line: 4123,
            col: 7,
        }
    }

    #[test]
    fn roundtrip() {
        let record = sample();
        assert_eq!(PanicRecord::parse(&record.serialize()), Ok(record));
    }

    #[test]
    fn fresh_roundtrip() {
        let mut file = [0u8; FILE_CAP];
        file[..3].copy_from_slice(b"lib");
        let record = PanicRecord {
            reset_flags: 0,
            consecutive_panics: 0,
            file,
            file_len: 3,
            line: 0,
            col: 0,
        };
        let parsed = PanicRecord::parse(&record.serialize()).unwrap();
        assert_eq!(parsed, record);
        assert_eq!(parsed.file_str(), Some("lib"));
    }

    #[test]
    fn detects_corruption() {
        let mut bytes = sample().serialize();
        bytes[2] ^= 0x01;
        assert_eq!(PanicRecord::parse(&bytes), Err(ParseError::BadCrc));
    }

    #[test]
    fn detects_bad_version() {
        let mut bytes = sample().serialize();
        bytes[0] = 9;
        // Recompute the CRC so the version check is what trips, not the CRC.
        let crc = crc::checksum32(&bytes[0..CRC_OFF]);
        bytes[CRC_OFF..CRC_OFF + 4].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(
            PanicRecord::parse(&bytes),
            Err(ParseError::UnsupportedVersion(9))
        );
    }

    #[test]
    fn blank_eeprom_fails_to_parse() {
        // A never-written EEPROM slot reads all `0xFF`. It must fail to parse
        // (CRC first, then version) so callers treat it as "no record".
        let bytes = [0xFFu8; RECORD_LEN];
        assert!(PanicRecord::parse(&bytes).is_err());
    }

    #[test]
    fn long_path_is_truncated() {
        let path = b"this/is/a/very/long/source/path/that/exceeds/the/48-byte/capacity.rs";
        assert!(path.len() > FILE_CAP);
        let mut file = [0u8; FILE_CAP];
        let len = store_path(path, &mut file);
        assert_eq!(usize::from(len), FILE_CAP);
        let (kept, _) = file.split_at(FILE_CAP);
        let (expected, _) = path.split_at(FILE_CAP);
        assert_eq!(kept, expected);
    }

    #[test]
    fn len_is_stable() {
        assert_eq!(sample().serialize().len(), RECORD_LEN);
    }
}
