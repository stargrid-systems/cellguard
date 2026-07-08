//! Packet layout on top of a COBS frame.
//!
//! A decoded frame is:
//!
//! ```text
//! Header{ id, kind, payload_len, header_crc } | payload | payload_crc
//! ```
//!
//! The header carries its own CRC-16 over `id`, `kind`, and `payload_len`. A
//! forwarding node validates just the header, and if `id` is not its own it can
//! relay the rest of the frame downstream without buffering or checking the
//! payload. The payload has a separate CRC-16 that only the destination checks.

use zerocopy::byteorder::little_endian::U16;
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned};

use crate::kind::Kind;

/// Length of the packet header in bytes.
pub const HEADER_LEN: usize = 6;

/// Length of the trailing payload CRC in bytes.
pub const PAYLOAD_CRC_LEN: usize = 2;

/// The fixed packet header.
#[derive(FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned, Clone, Copy, Debug)]
#[repr(C)]
pub struct Header {
    /// Node address this packet is addressed to or from.
    pub id: u8,
    /// Raw message kind. Validate with [`Kind::from_u8`].
    pub kind: u8,
    /// Length of the payload in bytes.
    pub payload_len: U16,
    /// CRC-16/MODBUS over `id`, `kind`, and `payload_len`.
    pub header_crc: U16,
}

impl Header {
    /// Builds a header with a correct `header_crc`.
    #[must_use]
    pub fn new(id: u8, kind: u8, payload_len: u16) -> Self {
        let header_crc = Self::compute_crc(id, kind, payload_len);
        Self {
            id,
            kind,
            payload_len: U16::new(payload_len),
            header_crc: U16::new(header_crc),
        }
    }

    /// Parses a header from the start of `bytes`, checking its CRC.
    ///
    /// Returns the header and the bytes that follow it. This is the fast path a
    /// forwarding node uses to read `id` without touching the payload.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Truncated`] if `bytes` is shorter than a header, or
    /// [`Error::BadHeaderCrc`] if the header CRC does not match.
    pub fn parse(bytes: &[u8]) -> Result<(Self, &[u8]), Error> {
        let (header, rest) = Self::ref_from_prefix(bytes).map_err(|_| Error::Truncated)?;
        if header.header_crc.get() != header.recomputed_crc() {
            return Err(Error::BadHeaderCrc);
        }
        Ok((*header, rest))
    }

    fn compute_crc(id: u8, kind: u8, payload_len: u16) -> u16 {
        let mut crc = crc::Crc16::new();
        crc.update(&[id, kind]);
        crc.update(&payload_len.to_le_bytes());
        crc.finalize()
    }

    fn recomputed_crc(self) -> u16 {
        Self::compute_crc(self.id, self.kind, self.payload_len.get())
    }
}

/// A parsed packet borrowing its payload from the decoded frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Packet<'a> {
    /// Node address.
    pub id: u8,
    /// Message kind.
    pub kind: Kind,
    /// Payload bytes.
    pub payload: &'a [u8],
}

impl<'a> Packet<'a> {
    /// Parses a full decoded frame into a packet, checking both CRCs.
    ///
    /// # Errors
    ///
    /// Returns [`Error`] if the frame is truncated, either CRC is wrong, or the
    /// kind is unknown.
    pub fn parse(frame: &'a [u8]) -> Result<Self, Error> {
        let (header, rest) = Header::parse(frame)?;
        let kind = Kind::from_u8(header.kind).ok_or(Error::UnknownKind(header.kind))?;
        let payload_len = usize::from(header.payload_len.get());

        let payload = rest.get(..payload_len).ok_or(Error::Truncated)?;
        let crc_bytes = rest
            .get(payload_len..payload_len + PAYLOAD_CRC_LEN)
            .ok_or(Error::Truncated)?;
        let expected = u16::from_le_bytes(crc_bytes.try_into().map_err(|_| Error::Truncated)?);
        if crc::checksum16(payload) != expected {
            return Err(Error::BadPayloadCrc);
        }

        Ok(Self {
            id: header.id,
            kind,
            payload,
        })
    }

    /// Writes a decoded frame for this packet into `out`, returning its length.
    ///
    /// The result is the pre-COBS frame. The caller COBS-encodes it onto the
    /// wire with [`crate::cobs::Encoder`].
    ///
    /// # Errors
    ///
    /// Returns [`Error::PayloadTooLarge`] if the payload exceeds `u16`, or
    /// [`Error::BufferTooSmall`] if `out` cannot hold the frame.
    pub fn write(id: u8, kind: Kind, payload: &[u8], out: &mut [u8]) -> Result<usize, Error> {
        let payload_len = u16::try_from(payload.len()).map_err(|_| Error::PayloadTooLarge)?;
        let total = HEADER_LEN + payload.len() + PAYLOAD_CRC_LEN;
        let slot = out.get_mut(..total).ok_or(Error::BufferTooSmall)?;

        let (head, tail) = slot.split_at_mut(HEADER_LEN);
        head.copy_from_slice(Header::new(id, kind.to_u8(), payload_len).as_bytes());
        let (body, crc) = tail.split_at_mut(payload.len());
        body.copy_from_slice(payload);
        crc.copy_from_slice(&crc::checksum16(payload).to_le_bytes());
        Ok(total)
    }
}

/// An error from building or parsing a packet.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum Error {
    /// The output buffer is too small for the frame.
    BufferTooSmall,
    /// The frame is shorter than the fields it claims to hold.
    Truncated,
    /// The header CRC did not match.
    BadHeaderCrc,
    /// The payload CRC did not match.
    BadPayloadCrc,
    /// The kind byte is not a known [`Kind`].
    UnknownKind(u8),
    /// The payload is larger than a `u16` length field can describe.
    PayloadTooLarge,
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall => f.write_str("output buffer too small"),
            Self::Truncated => f.write_str("frame truncated"),
            Self::BadHeaderCrc => f.write_str("header CRC mismatch"),
            Self::BadPayloadCrc => f.write_str("payload CRC mismatch"),
            Self::UnknownKind(k) => write!(f, "unknown kind {k}"),
            Self::PayloadTooLarge => f.write_str("payload too large"),
        }
    }
}

impl core::error::Error for Error {}

#[cfg(test)]
mod tests {
    use super::{Error, Header, Packet};
    use crate::kind::Kind;

    #[test]
    fn write_then_parse_roundtrips() {
        let payload = b"hello bus";
        let mut buf = [0u8; 64];
        let n = Packet::write(0x1A, Kind::ReadTemperature, payload, &mut buf).unwrap();
        let packet = Packet::parse(&buf[..n]).unwrap();
        assert_eq!(packet.id, 0x1A);
        assert_eq!(packet.kind, Kind::ReadTemperature);
        assert_eq!(packet.payload, payload);
    }

    #[test]
    fn empty_payload() {
        let mut buf = [0u8; 16];
        let n = Packet::write(2, Kind::Ack, &[], &mut buf).unwrap();
        let packet = Packet::parse(&buf[..n]).unwrap();
        assert_eq!(packet.payload, &[] as &[u8]);
        assert_eq!(packet.kind, Kind::Ack);
    }

    #[test]
    fn header_parse_exposes_id_without_payload() {
        let mut buf = [0u8; 64];
        let n = Packet::write(0x2B, Kind::ReadDeviceId, b"payload", &mut buf).unwrap();
        let (header, _rest) = Header::parse(&buf[..n]).unwrap();
        // A forwarder can route on id after only the header is validated.
        assert_eq!(header.id, 0x2B);
    }

    #[test]
    fn detects_corrupt_header() {
        let mut buf = [0u8; 64];
        let n = Packet::write(5, Kind::ReadTemperature, b"abc", &mut buf).unwrap();
        buf[0] ^= 0x01;
        assert_eq!(Packet::parse(&buf[..n]), Err(Error::BadHeaderCrc));
    }

    #[test]
    fn detects_corrupt_payload() {
        let mut buf = [0u8; 64];
        let n = Packet::write(5, Kind::ReadTemperature, b"abc", &mut buf).unwrap();
        buf[super::HEADER_LEN] ^= 0x01;
        assert_eq!(Packet::parse(&buf[..n]), Err(Error::BadPayloadCrc));
    }
}
