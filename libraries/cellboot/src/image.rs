//! The signed firmware image format.
//!
//! An image is a fixed-size [`ImageHeader`] followed by the raw payload. The
//! header carries routing metadata, a CRC-32 over the payload, and an
//! HMAC-SHA256 tag over header and payload. Signing and verification live in
//! `cellcore`, so this crate links no crypto.

use core::fmt;

/// Total length of the image header in bytes.
pub const HEADER_LEN: usize = 64;

/// [`HEADER_LEN`] as a `u32`, for offset arithmetic on 8-bit targets.
#[expect(
    clippy::cast_possible_truncation,
    reason = "the value is the constant 64, which always fits"
)]
pub const HEADER_LEN_U32: u32 = HEADER_LEN as u32;

const _: () = assert!(HEADER_LEN == HEADER_LEN_U32 as usize);

/// Number of leading header bytes covered by the authentication tag.
///
/// Every header field except the tag itself.
pub const MAC_PREFIX_LEN: usize = 32;

/// Magic bytes at the start of every header.
pub const MAGIC: [u8; 4] = *b"CGFW";

/// Header format version understood by this crate.
pub const FORMAT_VERSION: u8 = 1;

/// What kind of firmware an image carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ImageKind {
    /// The main application firmware.
    Application,
    /// Bootloader or programmer firmware.
    Bootloader,
}

impl ImageKind {
    #[must_use]
    const fn to_code(self) -> u8 {
        match self {
            Self::Application => 0,
            Self::Bootloader => 1,
        }
    }

    const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Application),
            1 => Some(Self::Bootloader),
            _ => None,
        }
    }
}

/// The storage region an image is destined for.
///
/// These mirror the labelled external EEPROM regions on the hardware.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Region {
    /// The application code region.
    ApplicationCode,
    /// The bootloader and settings region.
    Bootloader,
    /// The factory settings region.
    Factory,
    /// The cellagent application region.
    CellagentApp,
    /// The cellprog programmer application region. Flashed by the programmer
    /// onto itself through its self-update walker.
    CellprogApp,
}

impl Region {
    /// Returns the wire byte for this region.
    #[must_use]
    pub const fn to_code(self) -> u8 {
        match self {
            Self::ApplicationCode => 0,
            Self::Bootloader => 1,
            Self::Factory => 2,
            Self::CellagentApp => 3,
            Self::CellprogApp => 4,
        }
    }

    /// Parses a wire byte into a region.
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::ApplicationCode),
            1 => Some(Self::Bootloader),
            2 => Some(Self::Factory),
            3 => Some(Self::CellagentApp),
            4 => Some(Self::CellprogApp),
            _ => None,
        }
    }
}

/// The parsed contents of an image header.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ImageHeader {
    /// What kind of firmware this image carries.
    pub kind: ImageKind,
    /// The region the image is destined for.
    pub region: Region,
    /// Identifies the target device or board this image is built for.
    pub target_id: u16,
    /// Firmware version, informational only. Never used to reject an image,
    /// so downgrades stay possible.
    pub fw_version: u32,
    /// Length of the payload in bytes.
    pub payload_len: u32,
    /// CRC-32 of the payload, for corruption detection.
    pub payload_crc32: u32,
    /// HMAC-SHA256 tag over the header prefix and the payload.
    pub hmac: [u8; 32],
}

/// An error returned when a header cannot be parsed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ParseError {
    /// The magic bytes did not match [`MAGIC`].
    BadMagic,
    /// The header format version is not [`FORMAT_VERSION`].
    UnsupportedFormat(u8),
    /// The kind field held an unknown value.
    UnknownKind(u8),
    /// The region field held an unknown value.
    UnknownRegion(u8),
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadMagic => f.write_str("bad image magic"),
            Self::UnsupportedFormat(v) => write!(f, "unsupported header format version {v}"),
            Self::UnknownKind(v) => write!(f, "unknown image kind {v}"),
            Self::UnknownRegion(v) => write!(f, "unknown image region {v}"),
        }
    }
}

impl core::error::Error for ParseError {}

impl ImageHeader {
    /// Parses a header from its raw bytes.
    ///
    /// # Errors
    ///
    /// Returns a [`ParseError`] if the magic, format version, kind, or region
    /// fields are invalid.
    pub fn parse(bytes: &[u8; HEADER_LEN]) -> Result<Self, ParseError> {
        let magic: [u8; 4] = [bytes[0], bytes[1], bytes[2], bytes[3]];
        if magic != MAGIC {
            return Err(ParseError::BadMagic);
        }
        let format = bytes[4];
        if format != FORMAT_VERSION {
            return Err(ParseError::UnsupportedFormat(format));
        }
        let kind = ImageKind::from_code(bytes[5]).ok_or(ParseError::UnknownKind(bytes[5]))?;
        let region = Region::from_code(bytes[6]).ok_or(ParseError::UnknownRegion(bytes[6]))?;

        let fw_version = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        let payload_len = u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]);
        let payload_crc32 = u32::from_le_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);
        let target_id = u16::from_le_bytes([bytes[20], bytes[21]]);

        let mut hmac = [0u8; 32];
        hmac.copy_from_slice(&bytes[MAC_PREFIX_LEN..HEADER_LEN]);

        Ok(Self {
            kind,
            region,
            target_id,
            fw_version,
            payload_len,
            payload_crc32,
            hmac,
        })
    }

    /// Serializes the header to its canonical byte form.
    ///
    /// Reserved bytes are written as zero. Signers compute the tag over the
    /// first [`MAC_PREFIX_LEN`] bytes of this output followed by the payload.
    #[must_use]
    pub fn serialize(&self) -> [u8; HEADER_LEN] {
        let mut out = [0u8; HEADER_LEN];
        out[0..4].copy_from_slice(&MAGIC);
        out[4] = FORMAT_VERSION;
        out[5] = self.kind.to_code();
        out[6] = self.region.to_code();
        out[8..12].copy_from_slice(&self.fw_version.to_le_bytes());
        out[12..16].copy_from_slice(&self.payload_len.to_le_bytes());
        out[16..20].copy_from_slice(&self.payload_crc32.to_le_bytes());
        out[20..22].copy_from_slice(&self.target_id.to_le_bytes());
        out[MAC_PREFIX_LEN..HEADER_LEN].copy_from_slice(&self.hmac);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::{HEADER_LEN, ImageHeader, ImageKind, ParseError, Region};

    #[test]
    fn roundtrip_header() {
        let header = ImageHeader {
            kind: ImageKind::Bootloader,
            region: Region::Bootloader,
            target_id: 0xABCD,
            fw_version: 7,
            payload_len: 1000,
            payload_crc32: 0xDEAD_BEEF,
            hmac: [0x5Au8; 32],
        };
        let parsed = ImageHeader::parse(&header.serialize()).unwrap();
        assert_eq!(parsed, header);
    }

    #[test]
    fn region_codes_roundtrip() {
        let regions = [
            Region::ApplicationCode,
            Region::Bootloader,
            Region::Factory,
            Region::CellagentApp,
            Region::CellprogApp,
        ];
        for region in regions {
            assert_eq!(Region::from_code(region.to_code()), Some(region));
        }
        assert_eq!(Region::from_code(5), None);
    }

    #[test]
    fn rejects_bad_magic() {
        let mut bytes = [0u8; HEADER_LEN];
        bytes[4] = super::FORMAT_VERSION;
        assert_eq!(ImageHeader::parse(&bytes), Err(ParseError::BadMagic));
    }
}
