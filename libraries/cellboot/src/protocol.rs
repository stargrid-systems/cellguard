//! The wire protocol spoken over a transport during an update.
//!
//! The host is the initiator and the device is the responder. Each message is a
//! self-delimiting, CRC-protected frame:
//!
//! ```text
//! LEN(u16 LE) | OPCODE(u8) | BODY... | CRC32(u32 LE)
//! ```
//!
//! `LEN` counts everything after itself (opcode, body, and CRC). The CRC covers
//! the opcode and body. A transport reads two bytes to learn `LEN`, reads that
//! many more, then hands the whole frame to [`Command::decode`] or
//! [`Response::decode`].
//!
//! Encoding is buffer-based and allocation-free. The [`image`](crate::image)
//! header travels inside a `Begin`, and a payload chunk travels inside a `Data`.

use crate::crc32;
use crate::image::HEADER_LEN;
use crate::state::{PersistentState, STATE_LEN};

const OP_PROBE: u8 = 0x01;
const OP_BEGIN: u8 = 0x02;
const OP_DATA: u8 = 0x03;
const OP_COMMIT: u8 = 0x04;
const OP_ABORT: u8 = 0x05;

const OP_STATUS: u8 = 0x81;
const OP_ACK: u8 = 0x82;
const OP_NACK: u8 = 0x83;

const LEN_FIELD: usize = 2;
const CRC_FIELD: usize = 4;

/// A command sent by the host to the device.
///
/// `Data` borrows its chunk from the decode buffer, so a command does not copy
/// the payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command<'a> {
    /// Ask the device to report its [`PersistentState`].
    Probe,
    /// Begin an update carrying the raw image header.
    Begin {
        /// The raw image header bytes.
        header: [u8; HEADER_LEN],
    },
    /// Deliver a payload chunk at a byte offset within the payload.
    Data {
        /// Offset of this chunk within the payload.
        offset: u32,
        /// The chunk bytes.
        chunk: &'a [u8],
    },
    /// Finish the update: verify and stage the image.
    Commit,
    /// Abandon the current update.
    Abort,
}

impl<'a> Command<'a> {
    /// Encodes the command as a frame into `out`, returning its length.
    ///
    /// # Errors
    ///
    /// Returns [`ProtocolError::BufferTooSmall`] if `out` cannot hold the frame.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, ProtocolError> {
        match self {
            Self::Probe => encode_frame(OP_PROBE, &[], &[], out),
            Self::Begin { header } => encode_frame(OP_BEGIN, header, &[], out),
            Self::Data { offset, chunk } => encode_frame(OP_DATA, &offset.to_le_bytes(), chunk, out),
            Self::Commit => encode_frame(OP_COMMIT, &[], &[], out),
            Self::Abort => encode_frame(OP_ABORT, &[], &[], out),
        }
    }

    /// Decodes a command from a complete frame.
    ///
    /// # Errors
    ///
    /// Returns [`ProtocolError`] if the frame is malformed, the CRC is wrong, or
    /// the opcode is not a command.
    pub fn decode(frame: &'a [u8]) -> Result<Self, ProtocolError> {
        let (opcode, body) = split_frame(frame)?;
        match opcode {
            OP_PROBE => Ok(Self::Probe),
            OP_COMMIT => Ok(Self::Commit),
            OP_ABORT => Ok(Self::Abort),
            OP_BEGIN => {
                let mut header = [0u8; HEADER_LEN];
                let src = body.get(..HEADER_LEN).ok_or(ProtocolError::Truncated)?;
                header.copy_from_slice(src);
                Ok(Self::Begin { header })
            }
            OP_DATA => {
                let offset_bytes = body.get(..4).ok_or(ProtocolError::Truncated)?;
                let offset = u32::from_le_bytes([
                    offset_bytes.first().copied().unwrap_or(0),
                    offset_bytes.get(1).copied().unwrap_or(0),
                    offset_bytes.get(2).copied().unwrap_or(0),
                    offset_bytes.get(3).copied().unwrap_or(0),
                ]);
                let chunk = body.get(4..).ok_or(ProtocolError::Truncated)?;
                Ok(Self::Data { offset, chunk })
            }
            other => Err(ProtocolError::UnknownOpcode(other)),
        }
    }
}

/// A response sent by the device to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Response {
    /// The device's current state, in reply to `Probe`.
    Status(PersistentState),
    /// The command succeeded. `next_offset` is the next payload offset the
    /// device expects, so the host can track progress and resync.
    Ack {
        /// Next payload offset the device expects.
        next_offset: u32,
    },
    /// The command was rejected.
    Nack(NackReason),
}

impl Response {
    /// Encodes the response as a frame into `out`, returning its length.
    ///
    /// # Errors
    ///
    /// Returns [`ProtocolError::BufferTooSmall`] if `out` cannot hold the frame.
    pub fn encode(&self, out: &mut [u8]) -> Result<usize, ProtocolError> {
        match self {
            Self::Status(state) => encode_frame(OP_STATUS, &state.serialize(), &[], out),
            Self::Ack { next_offset } => encode_frame(OP_ACK, &next_offset.to_le_bytes(), &[], out),
            Self::Nack(reason) => encode_frame(OP_NACK, &[reason.to_code()], &[], out),
        }
    }

    /// Decodes a response from a complete frame.
    ///
    /// # Errors
    ///
    /// Returns [`ProtocolError`] if the frame is malformed, the CRC is wrong, or
    /// the opcode is not a response.
    pub fn decode(frame: &[u8]) -> Result<Self, ProtocolError> {
        let (opcode, body) = split_frame(frame)?;
        match opcode {
            OP_STATUS => {
                let mut bytes = [0u8; STATE_LEN];
                let src = body.get(..STATE_LEN).ok_or(ProtocolError::Truncated)?;
                bytes.copy_from_slice(src);
                let state = PersistentState::parse(&bytes).map_err(|_| ProtocolError::BadBody)?;
                Ok(Self::Status(state))
            }
            OP_ACK => {
                let b = body.get(..4).ok_or(ProtocolError::Truncated)?;
                let next_offset = u32::from_le_bytes([
                    b.first().copied().unwrap_or(0),
                    b.get(1).copied().unwrap_or(0),
                    b.get(2).copied().unwrap_or(0),
                    b.get(3).copied().unwrap_or(0),
                ]);
                Ok(Self::Ack { next_offset })
            }
            OP_NACK => {
                let code = body.first().copied().ok_or(ProtocolError::Truncated)?;
                let reason = NackReason::from_code(code).ok_or(ProtocolError::BadBody)?;
                Ok(Self::Nack(reason))
            }
            other => Err(ProtocolError::UnknownOpcode(other)),
        }
    }
}

/// The reason a command was rejected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum NackReason {
    /// The command could not be parsed or made no sense here.
    Malformed,
    /// The image is not built for this device.
    WrongTarget,
    /// The command is not valid in the current session state.
    BadState,
    /// A `Data` chunk arrived at an unexpected offset.
    OutOfOrder,
    /// The image does not fit in the target storage region.
    TooLarge,
    /// A storage operation failed.
    StorageError,
    /// The image failed verification at commit.
    VerifyFailed,
}

impl NackReason {
    const fn to_code(self) -> u8 {
        match self {
            Self::Malformed => 0,
            Self::WrongTarget => 1,
            Self::BadState => 2,
            Self::OutOfOrder => 3,
            Self::TooLarge => 4,
            Self::StorageError => 5,
            Self::VerifyFailed => 6,
        }
    }

    const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Malformed),
            1 => Some(Self::WrongTarget),
            2 => Some(Self::BadState),
            3 => Some(Self::OutOfOrder),
            4 => Some(Self::TooLarge),
            5 => Some(Self::StorageError),
            6 => Some(Self::VerifyFailed),
            _ => None,
        }
    }
}

/// An error from encoding or decoding a frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ProtocolError {
    /// The output buffer is too small for the frame.
    BufferTooSmall,
    /// The frame is shorter than its own length field claims.
    Truncated,
    /// The length field is too small to hold a valid frame.
    BadLength,
    /// The frame CRC did not match.
    BadCrc,
    /// The opcode is not valid for the decoded direction.
    UnknownOpcode(u8),
    /// The body could not be parsed.
    BadBody,
}

impl core::fmt::Display for ProtocolError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::BufferTooSmall => f.write_str("output buffer too small"),
            Self::Truncated => f.write_str("frame truncated"),
            Self::BadLength => f.write_str("bad frame length"),
            Self::BadCrc => f.write_str("frame CRC mismatch"),
            Self::UnknownOpcode(op) => write!(f, "unknown opcode {op:#04x}"),
            Self::BadBody => f.write_str("bad frame body"),
        }
    }
}

impl core::error::Error for ProtocolError {}

/// Returns the total frame length given the two `LEN` bytes read from the wire.
///
/// A transport reads two bytes, calls this, then reads that many more bytes to
/// have the complete frame.
#[must_use]
pub fn frame_len(len_field: [u8; 2]) -> usize {
    LEN_FIELD + usize::from(u16::from_le_bytes(len_field))
}

fn encode_frame(opcode: u8, body_a: &[u8], body_b: &[u8], out: &mut [u8]) -> Result<usize, ProtocolError> {
    let payload = 1 + body_a.len() + body_b.len() + CRC_FIELD;
    let total = LEN_FIELD + payload;
    let slot = out.get_mut(..total).ok_or(ProtocolError::BufferTooSmall)?;

    let len_value = u16::try_from(payload).map_err(|_| ProtocolError::BadLength)?;

    let mut crc = crc32::Crc32::new();
    crc.update(&[opcode]);
    crc.update(body_a);
    crc.update(body_b);
    let crc = crc.finalize();

    let mut writer = Writer::new(slot);
    writer.put(&len_value.to_le_bytes())?;
    writer.put(&[opcode])?;
    writer.put(body_a)?;
    writer.put(body_b)?;
    writer.put(&crc.to_le_bytes())?;
    Ok(total)
}

/// Splits a complete frame into its opcode and body, checking the CRC.
fn split_frame(frame: &[u8]) -> Result<(u8, &[u8]), ProtocolError> {
    let len_bytes = frame.get(..LEN_FIELD).ok_or(ProtocolError::Truncated)?;
    let claimed = usize::from(u16::from_le_bytes([
        len_bytes.first().copied().unwrap_or(0),
        len_bytes.get(1).copied().unwrap_or(0),
    ]));
    let payload = frame.get(LEN_FIELD..LEN_FIELD + claimed).ok_or(ProtocolError::Truncated)?;
    if payload.len() < 1 + CRC_FIELD {
        return Err(ProtocolError::BadLength);
    }
    let split = payload.len() - CRC_FIELD;
    let opcode_and_body = payload.get(..split).ok_or(ProtocolError::BadLength)?;
    let crc_bytes = payload.get(split..).ok_or(ProtocolError::BadLength)?;
    let expected = u32::from_le_bytes([
        crc_bytes.first().copied().unwrap_or(0),
        crc_bytes.get(1).copied().unwrap_or(0),
        crc_bytes.get(2).copied().unwrap_or(0),
        crc_bytes.get(3).copied().unwrap_or(0),
    ]);
    if crc32::checksum(opcode_and_body) != expected {
        return Err(ProtocolError::BadCrc);
    }
    let opcode = opcode_and_body.first().copied().ok_or(ProtocolError::BadLength)?;
    let body = opcode_and_body.get(1..).ok_or(ProtocolError::BadLength)?;
    Ok((opcode, body))
}

/// A minimal bounds-checked byte writer used during encoding.
struct Writer<'a> {
    buf: &'a mut [u8],
    pos: usize,
}

impl<'a> Writer<'a> {
    const fn new(buf: &'a mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    fn put(&mut self, bytes: &[u8]) -> Result<(), ProtocolError> {
        let end = self.pos.checked_add(bytes.len()).ok_or(ProtocolError::BufferTooSmall)?;
        let slot = self.buf.get_mut(self.pos..end).ok_or(ProtocolError::BufferTooSmall)?;
        slot.copy_from_slice(bytes);
        self.pos = end;
        Ok(())
    }
}

#[cfg(test)]
#[expect(
    clippy::indexing_slicing,
    reason = "tests build and inspect fixed-size frame buffers with in-range indices"
)]
mod tests {
    use std::vec::Vec;

    use super::{Command, NackReason, ProtocolError, Response, frame_len};
    use crate::image::HEADER_LEN;
    use crate::state::PersistentState;

    fn roundtrip_command(cmd: &Command) {
        let mut buf = [0u8; 256];
        let n = cmd.encode(&mut buf).unwrap();
        assert_eq!(frame_len([buf[0], buf[1]]), n);
        assert_eq!(&Command::decode(&buf[..n]).unwrap(), cmd);
    }

    #[test]
    fn command_roundtrips() {
        roundtrip_command(&Command::Probe);
        roundtrip_command(&Command::Commit);
        roundtrip_command(&Command::Abort);
        roundtrip_command(&Command::Begin { header: [0xA5; HEADER_LEN] });
        roundtrip_command(&Command::Data { offset: 0x1234_5678, chunk: b"payload bytes" });
    }

    #[test]
    fn response_roundtrips() {
        let mut buf = [0u8; 256];

        let ack = Response::Ack { next_offset: 4096 };
        let n = ack.encode(&mut buf).unwrap();
        assert_eq!(Response::decode(&buf[..n]).unwrap(), ack);

        let nack = Response::Nack(NackReason::VerifyFailed);
        let n = nack.encode(&mut buf).unwrap();
        assert_eq!(Response::decode(&buf[..n]).unwrap(), nack);

        let status = Response::Status(PersistentState::new(0x0102_0304));
        let n = status.encode(&mut buf).unwrap();
        assert_eq!(Response::decode(&buf[..n]).unwrap(), status);
    }

    #[test]
    fn detects_crc_error() {
        let mut buf = [0u8; 256];
        let n = Command::Probe.encode(&mut buf).unwrap();
        buf[3] ^= 0xFF;
        assert_eq!(Command::decode(&buf[..n]), Err(ProtocolError::BadCrc));
    }

    #[test]
    fn rejects_small_buffer() {
        let mut buf = [0u8; 4];
        assert_eq!(
            Command::Begin { header: [0; HEADER_LEN] }.encode(&mut buf),
            Err(ProtocolError::BufferTooSmall)
        );
    }

    #[test]
    fn data_chunk_borrows_input() {
        let payload: Vec<u8> = (0u8..100).collect();
        let mut buf = [0u8; 256];
        let n = Command::Data { offset: 0, chunk: &payload }.encode(&mut buf).unwrap();
        match Command::decode(&buf[..n]).unwrap() {
            Command::Data { offset, chunk } => {
                assert_eq!(offset, 0);
                assert_eq!(chunk, payload.as_slice());
            }
            other => panic!("expected Data, got {other:?}"),
        }
    }
}
