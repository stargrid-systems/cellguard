//! Streaming image verification, plus host-side signing behind the `sign`
//! feature.
//!
//! [`Verifier`] streams the header and payload through a [`Mac`] and a
//! [`Crc32`], so an image staged in external storage can be checked in chunks
//! without ever holding it whole in RAM. The host-side counterpart (`sign`,
//! behind the `sign` feature) produces a signed header. A device never signs,
//! so firmware links none of it.
//!
//! Both work on the [`ImageHeader`] format defined in `cellboot`.

use core::fmt;

use cellboot::image::{ImageHeader, ParseError, HEADER_LEN, MAC_PREFIX_LEN};
use crc::Crc32;

use crate::update::mac::{ct_eq, Mac};

/// Signs `payload` and returns the complete header bytes.
///
/// The payload length and CRC-32 are derived from `payload`, then the tag is
/// computed over the header prefix followed by the payload using the keyed
/// `mac`. The `payload_len`, `payload_crc32`, and `hmac` fields of `header` are
/// ignored on input and filled in the result.
///
/// This is the host-side counterpart to [`Verifier`]. It needs the whole
/// payload at once because the CRC lives inside the signed prefix, so a device
/// never signs, it only verifies.
///
/// # Errors
///
/// Returns [`SignError::PayloadTooLarge`] if the payload does not fit in a
/// `u32` length field.
#[cfg(any(test, feature = "sign"))]
pub fn sign<M: Mac>(
    mut header: ImageHeader,
    mut mac: M,
    payload: &[u8],
) -> Result<[u8; HEADER_LEN], SignError> {
    header.payload_len = u32::try_from(payload.len()).map_err(|_| SignError::PayloadTooLarge)?;
    header.payload_crc32 = crc::checksum32(payload);
    let prefix = header.serialize();
    mac.update(prefix.split_at(MAC_PREFIX_LEN).0);
    mac.update(payload);
    header.hmac = mac.finalize();
    Ok(header.serialize())
}

/// An error returned when an image cannot be signed.
#[cfg(any(test, feature = "sign"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum SignError {
    /// The payload is larger than a `u32` length field can describe.
    PayloadTooLarge,
}

#[cfg(any(test, feature = "sign"))]
impl fmt::Display for SignError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::PayloadTooLarge => f.write_str("payload too large"),
        }
    }
}

#[cfg(any(test, feature = "sign"))]
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
    pub fn new(
        mut mac: M,
        header_bytes: &[u8; HEADER_LEN],
    ) -> Result<(ImageHeader, Self), ParseError> {
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
        self.fed = self
            .fed
            .saturating_add(chunk.len().try_into().unwrap_or(u32::MAX));
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
    use cellboot::image::{ImageHeader, ImageKind, Region, HEADER_LEN};
    use hmac_sha256::HMAC;

    use super::{sign, Verifier, VerifyError};

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
        sign(header, HMAC::new(KEY), payload).unwrap()
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
