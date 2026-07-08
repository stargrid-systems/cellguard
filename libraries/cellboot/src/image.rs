//! The signed firmware image format.
//!
//! An image is a fixed-size [`ImageHeader`] followed by the raw firmware
//! payload. The header carries the metadata needed to route and check the
//! image, a CRC-32 over the payload for corruption detection, and an
//! HMAC-SHA256 tag over the header and payload for authenticity.
//!
//! [`Verifier`] streams the header and payload through a [`Mac`] and a
//! [`Crc32`], so an image staged in external storage can be checked in chunks
//! without ever holding it whole in RAM.

use core::fmt;

use crc::Crc32;
use crate::mac::{Mac, ct_eq};

/// Total length of the image header in bytes.
pub const HEADER_LEN: usize = 64;

/// Number of leading header bytes covered by the authentication tag.
///
/// The tag is computed over these bytes followed by the payload. It is every
/// header field except the tag itself.
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
}

impl Region {
    #[must_use]
    pub(crate) const fn to_code(self) -> u8 {
        match self {
            Self::ApplicationCode => 0,
            Self::Bootloader => 1,
            Self::Factory => 2,
        }
    }

    pub(crate) const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::ApplicationCode),
            1 => Some(Self::Bootloader),
            2 => Some(Self::Factory),
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
    /// Firmware version, informational only. It is never used to reject an
    /// image, so a downgrade to a known-good version is always allowed.
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
    /// Reserved bytes are written as zero. Callers that build and sign images
    /// must compute the tag over the first [`MAC_PREFIX_LEN`] bytes of this
    /// output followed by the payload.
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

    /// Signs `payload` and returns the complete header bytes.
    ///
    /// The payload length and CRC-32 are derived from `payload`, then the tag
    /// is computed over the header prefix followed by the payload using the
    /// keyed `mac`. The `payload_len`, `payload_crc32`, and `hmac` fields of
    /// `self` are ignored on input and filled in the result.
    ///
    /// This is the host-side counterpart to [`Verifier`]. It needs the whole
    /// payload at once because the CRC lives inside the signed prefix, so a
    /// device never signs, it only verifies.
    ///
    /// # Errors
    ///
    /// Returns [`SignError::PayloadTooLarge`] if the payload does not fit in a
    /// `u32` length field.
    pub fn sign<M: Mac>(mut self, mut mac: M, payload: &[u8]) -> Result<[u8; HEADER_LEN], SignError> {
        self.payload_len = u32::try_from(payload.len()).map_err(|_| SignError::PayloadTooLarge)?;
        self.payload_crc32 = crc::checksum32(payload);
        let prefix = self.serialize();
        mac.update(prefix.split_at(MAC_PREFIX_LEN).0);
        mac.update(payload);
        self.hmac = mac.finalize();
        Ok(self.serialize())
    }
}

/// An error returned when an image cannot be signed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SignError {
    /// The payload is larger than a `u32` length field can describe.
    PayloadTooLarge,
}

impl fmt::Display for SignError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PayloadTooLarge => f.write_str("payload too large"),
        }
    }
}

impl core::error::Error for SignError {}

/// The reason an image failed verification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum VerifyError {
    /// The number of payload bytes fed did not match the header.
    WrongLength,
    /// The payload CRC-32 did not match the header.
    CorruptPayload,
    /// The authentication tag did not match.
    BadTag,
}

impl fmt::Display for VerifyError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongLength => f.write_str("payload length mismatch"),
            Self::CorruptPayload => f.write_str("payload CRC mismatch"),
            Self::BadTag => f.write_str("authentication tag mismatch"),
        }
    }
}

impl core::error::Error for VerifyError {}

/// Streams an image through a [`Mac`] and a [`Crc32`] to check it.
///
/// Construct it from the raw header bytes and a freshly keyed MAC, feed the
/// payload with [`Verifier::feed`], then call [`Verifier::finish`]. The payload
/// may be fed in any number of chunks.
pub struct Verifier<M: Mac> {
    mac: M,
    crc: Crc32,
    expected_tag: [u8; 32],
    expected_crc: u32,
    payload_len: u32,
    fed: u32,
}

impl<M: Mac> Verifier<M> {
    /// Starts verifying an image.
    ///
    /// Parses `header_bytes`, primes `mac` with the header prefix, and returns
    /// the parsed header alongside the verifier.
    ///
    /// # Errors
    ///
    /// Returns a [`ParseError`] if the header cannot be parsed.
    pub fn new(mut mac: M, header_bytes: &[u8; HEADER_LEN]) -> Result<(ImageHeader, Self), ParseError> {
        let header = ImageHeader::parse(header_bytes)?;
        mac.update(&header_bytes[..MAC_PREFIX_LEN]);
        let verifier = Self {
            mac,
            crc: Crc32::new(),
            expected_tag: header.hmac,
            expected_crc: header.payload_crc32,
            payload_len: header.payload_len,
            fed: 0,
        };
        Ok((header, verifier))
    }

    /// Feeds a chunk of payload bytes.
    pub fn feed(&mut self, chunk: &[u8]) {
        self.mac.update(chunk);
        self.crc.update(chunk);
        self.fed = self.fed.saturating_add(chunk.len().try_into().unwrap_or(u32::MAX));
    }

    /// Consumes the verifier and reports whether the image is valid.
    ///
    /// # Errors
    ///
    /// Returns a [`VerifyError`] if the fed length, the payload CRC, or the
    /// authentication tag did not match the header.
    pub fn finish(self) -> Result<(), VerifyError> {
        if self.fed != self.payload_len {
            return Err(VerifyError::WrongLength);
        }
        // The tag is authoritative, but the CRC gives a cheaper corruption
        // signal and is checked first.
        if self.crc.finalize() != self.expected_crc {
            return Err(VerifyError::CorruptPayload);
        }
        if ct_eq(&self.mac.finalize(), &self.expected_tag) {
            Ok(())
        } else {
            Err(VerifyError::BadTag)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{HEADER_LEN, ImageHeader, ImageKind, ParseError, Region, VerifyError, Verifier};
    use hmac_sha256::HMAC;

    const KEY: &[u8] = b"unit-test-shared-key";

    #[expect(
        clippy::cast_possible_truncation,
        reason = "index stays below 200 which fits in a u8"
    )]
    fn ramp() -> [u8; 200] {
        core::array::from_fn(|i| i as u8)
    }

    fn build_signed(payload: &[u8]) -> [u8; HEADER_LEN] {
        let header = ImageHeader {
            kind: ImageKind::Application,
            region: Region::ApplicationCode,
            target_id: 0x1234,
            fw_version: 42,
            payload_len: 0,
            payload_crc32: 0,
            hmac: [0u8; 32],
        };
        header.sign(HMAC::new(KEY), payload).unwrap()
    }

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
    fn rejects_bad_magic() {
        let mut bytes = [0u8; HEADER_LEN];
        bytes[4] = super::FORMAT_VERSION;
        assert_eq!(ImageHeader::parse(&bytes), Err(ParseError::BadMagic));
    }

    #[test]
    fn verifies_good_image() {
        let payload = ramp();
        let header_bytes = build_signed(&payload);
        let (header, mut verifier) = Verifier::new(HMAC::new(KEY), &header_bytes).unwrap();
        assert_eq!(header.target_id, 0x1234);
        for chunk in payload.chunks(7) {
            verifier.feed(chunk);
        }
        assert_eq!(verifier.finish(), Ok(()));
    }

    #[test]
    fn detects_tampered_payload() {
        let payload = ramp();
        let header_bytes = build_signed(&payload);
        let (_, mut verifier) = Verifier::new(HMAC::new(KEY), &header_bytes).unwrap();
        let mut tampered = payload;
        if let Some(first) = tampered.first_mut() {
            *first ^= 0x01;
        }
        verifier.feed(&tampered);
        // CRC catches the flip before the tag does.
        assert_eq!(verifier.finish(), Err(VerifyError::CorruptPayload));
    }

    #[test]
    fn detects_short_payload() {
        let payload = ramp();
        let header_bytes = build_signed(&payload);
        let (_, mut verifier) = Verifier::new(HMAC::new(KEY), &header_bytes).unwrap();
        verifier.feed(payload.get(..100).unwrap());
        assert_eq!(verifier.finish(), Err(VerifyError::WrongLength));
    }

    #[test]
    fn detects_wrong_key() {
        let payload = ramp();
        let header_bytes = build_signed(&payload);
        let (_, mut verifier) = Verifier::new(HMAC::new(b"wrong-key"), &header_bytes).unwrap();
        for chunk in payload.chunks(7) {
            verifier.feed(chunk);
        }
        assert_eq!(verifier.finish(), Err(VerifyError::BadTag));
    }
}
