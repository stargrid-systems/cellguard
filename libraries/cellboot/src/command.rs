//! Bootloader commands and responses, and their mapping to bus packets.
//!
//! The update session works on these semantic types. This module maps them to
//! and from [`cellguard_protocol`] packets using the bootloader [`Kind`]s, so
//! the wire framing (COBS, CRCs) lives entirely in the protocol crate.

use cellguard_protocol::{Error as PacketError, Kind, Packet};

use crate::image::HEADER_LEN;
use crate::state::PersistentState;

/// A command from the host to the update agent.
///
/// `Data` borrows its chunk from the packet, so mapping a command does not copy
/// the payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command<'a> {
    /// Ask the agent to report its [`PersistentState`].
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
    /// Maps a parsed packet to a bootloader command.
    ///
    /// # Errors
    ///
    /// Returns [`MapError::NotBootCommand`] if the packet is not a bootloader
    /// request, or [`MapError::BadPayload`] if the payload is the wrong shape.
    pub fn from_packet(packet: Packet<'a>) -> Result<Self, MapError> {
        match packet.kind {
            Kind::BootProbe => Ok(Self::Probe),
            Kind::BootCommit => Ok(Self::Commit),
            Kind::BootAbort => Ok(Self::Abort),
            Kind::BootBegin => {
                let header = packet.payload.try_into().map_err(|_| MapError::BadPayload)?;
                Ok(Self::Begin { header })
            }
            Kind::BootData => {
                let offset_bytes = packet.payload.get(..4).ok_or(MapError::BadPayload)?;
                let offset =
                    u32::from_le_bytes(offset_bytes.try_into().map_err(|_| MapError::BadPayload)?);
                let chunk = packet.payload.get(4..).ok_or(MapError::BadPayload)?;
                Ok(Self::Data { offset, chunk })
            }
            other => Err(MapError::NotBootCommand(other)),
        }
    }
}

/// A response from the update agent to the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Response {
    /// The agent's current state, in reply to [`Command::Probe`].
    Status(PersistentState),
    /// The command succeeded. `next_offset` is the next payload offset the
    /// agent expects, so the host can track progress and resync.
    Ack {
        /// Next payload offset the agent expects.
        next_offset: u32,
    },
    /// The command was rejected.
    Nack(NackReason),
}

impl Response {
    /// Writes this response as a packet frame into `out`, returning its length.
    ///
    /// The result is the pre-COBS frame; the caller COBS-encodes it with
    /// [`cellguard_protocol::Encoder`].
    ///
    /// # Errors
    ///
    /// Returns a [`PacketError`] if `out` is too small.
    pub fn to_packet(self, id: u8, out: &mut [u8]) -> Result<usize, PacketError> {
        match self {
            Self::Status(state) => Packet::write(id, Kind::BootStatus, &state.serialize(), out),
            Self::Ack { next_offset } => {
                Packet::write(id, Kind::BootAck, &next_offset.to_le_bytes(), out)
            }
            Self::Nack(reason) => Packet::write(id, Kind::BootNack, &[reason.to_code()], out),
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
    /// Returns the wire byte for this reason.
    #[must_use]
    pub const fn to_code(self) -> u8 {
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

    /// Parses a wire byte into a reason.
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
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

/// An error mapping a packet to a bootloader command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum MapError {
    /// The packet kind is not a bootloader request.
    NotBootCommand(Kind),
    /// The payload did not match the command it belongs to.
    BadPayload,
}

impl core::fmt::Display for MapError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::NotBootCommand(kind) => write!(f, "not a bootloader command: {kind:?}"),
            Self::BadPayload => f.write_str("bad command payload"),
        }
    }
}

impl core::error::Error for MapError {}

#[cfg(test)]
mod tests {
    use cellguard_protocol::{Kind, Packet};

    use super::{Command, MapError, NackReason, Response};
    use crate::image::HEADER_LEN;
    use crate::state::PersistentState;

    fn command_from_bytes<'a>(kind: Kind, payload: &[u8], buf: &'a mut [u8]) -> Command<'a> {
        let n = Packet::write(1, kind, payload, buf).unwrap();
        let packet = Packet::parse(&buf[..n]).unwrap();
        Command::from_packet(packet).unwrap()
    }

    #[test]
    fn maps_simple_commands() {
        let mut buf = [0u8; 128];
        assert_eq!(command_from_bytes(Kind::BootProbe, &[], &mut buf), Command::Probe);
        assert_eq!(command_from_bytes(Kind::BootCommit, &[], &mut buf), Command::Commit);
        assert_eq!(command_from_bytes(Kind::BootAbort, &[], &mut buf), Command::Abort);
    }

    #[test]
    fn maps_begin_and_data() {
        let mut buf = [0u8; 256];
        let header = [0xA5u8; HEADER_LEN];
        assert_eq!(
            command_from_bytes(Kind::BootBegin, &header, &mut buf),
            Command::Begin { header }
        );

        let mut data_payload = [0u8; 12];
        data_payload[..4].copy_from_slice(&0x1234_5678u32.to_le_bytes());
        data_payload[4..].copy_from_slice(b"payload!");
        let mut buf2 = [0u8; 64];
        match command_from_bytes(Kind::BootData, &data_payload, &mut buf2) {
            Command::Data { offset, chunk } => {
                assert_eq!(offset, 0x1234_5678);
                assert_eq!(chunk, b"payload!");
            }
            other => panic!("expected Data, got {other:?}"),
        }
    }

    #[test]
    fn rejects_non_boot_kind() {
        let mut buf = [0u8; 64];
        let n = Packet::write(1, Kind::ReadTemperature, &[], &mut buf).unwrap();
        let packet = Packet::parse(&buf[..n]).unwrap();
        assert_eq!(
            Command::from_packet(packet),
            Err(MapError::NotBootCommand(Kind::ReadTemperature))
        );
    }

    #[test]
    fn response_status_roundtrips() {
        let mut buf = [0u8; 128];
        let state = PersistentState::new(0x0102_0304);
        let n = Response::Status(state).to_packet(9, &mut buf).unwrap();
        let packet = Packet::parse(&buf[..n]).unwrap();
        assert_eq!(packet.id, 9);
        assert_eq!(packet.kind, Kind::BootStatus);
        assert_eq!(PersistentState::parse(&packet.payload.try_into().unwrap()).unwrap(), state);
    }

    #[test]
    fn response_ack_and_nack_roundtrip() {
        let mut buf = [0u8; 64];
        let n = Response::Ack { next_offset: 4096 }.to_packet(2, &mut buf).unwrap();
        let packet = Packet::parse(&buf[..n]).unwrap();
        assert_eq!(packet.kind, Kind::BootAck);
        assert_eq!(u32::from_le_bytes(packet.payload.try_into().unwrap()), 4096);

        let n = Response::Nack(NackReason::VerifyFailed).to_packet(2, &mut buf).unwrap();
        let packet = Packet::parse(&buf[..n]).unwrap();
        assert_eq!(packet.kind, Kind::BootNack);
        assert_eq!(NackReason::from_code(packet.payload[0]), Some(NackReason::VerifyFailed));
    }
}
