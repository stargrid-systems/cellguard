//! The transactional programming session over the local `UART_PROG` link.
//!
//! The `cellprog` MCU reaches both its cellcore UART and the UPDI targets
//! through one analog mux (channel 0 is the UART, the rest are UPDI lines),
//! so a transparent byte pipe is impossible. The programmer services one
//! command per transaction: receive it, switch the mux, run one UPDI
//! operation, switch back, reply. Exactly one command may be in flight.
//! Commands sent while the mux is on a UPDI channel are electrically lost.
//!
//! A session is [`SessionCmd::Begin`] (chip-erase and enter programming
//! mode), any number of [`SessionCmd::PageWrite`], optional read-back via
//! [`SessionCmd::PageRead`], then [`SessionCmd::End`]. Page commands before a
//! successful `Begin` are rejected with [`SessionStatus::BadState`]: writing
//! un-erased flash corrupts it.
//!
//! Frames are lean because the link is point-to-point: `[cmd][body][crc16]`,
//! COBS-encoded, with no address or kind byte.

/// Maximum data bytes carried by one page command or reply.
pub const PAGE_MAX: usize = 64;

/// Worst-case COBS-encoded size of the largest command (`PageWrite`).
pub const MAX_COMMAND_WIRE: usize = crate::max_encoded_len(1 + 2 + PAGE_MAX + CRC_LEN);

/// Worst-case COBS-encoded size of the largest reply (`PageData`).
pub const MAX_REPLY_WIRE: usize = crate::max_encoded_len(1 + 3 + PAGE_MAX + CRC_LEN);

/// Length of the frame CRC in bytes.
const CRC_LEN: usize = 2;

/// Which target a session should program.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionTarget {
    /// The `cellagent` balancer MCU, reached over UPDI mux channel 3.
    Cellagent,
    /// The `cellcore` MCU over mux channel 1. Reserved: this programmer
    /// answers [`SessionStatus::NotSupported`].
    Cellcore,
}

impl SessionTarget {
    /// Returns the wire byte for this target.
    #[must_use]
    pub const fn to_code(self) -> u8 {
        match self {
            Self::Cellagent => 0,
            Self::Cellcore => 1,
        }
    }

    /// Parses a wire byte into a target.
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Cellagent),
            1 => Some(Self::Cellcore),
            _ => None,
        }
    }
}

/// The outcome of one session command, reported in every reply.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionStatus {
    /// The command completed.
    Ok,
    /// The requested target is not supported by this programmer.
    NotSupported,
    /// A page command arrived outside a session (no `Begin`, or after `End`).
    BadState,
    /// The target did not respond over UPDI.
    NotAlive,
    /// The target is locked, or rejected the programming key.
    Locked,
    /// An NVM operation stayed busy past the programmer's poll bound.
    Busy,
    /// The NVM controller reported a write error.
    NvmError,
    /// The flash address was misaligned or out of range.
    InvalidAddr,
    /// Entering or erasing never completed within the poll bound.
    Timeout,
    /// The UPDI transport itself failed.
    Link,
}

impl SessionStatus {
    /// Returns the wire byte for this status.
    #[must_use]
    pub const fn to_code(self) -> u8 {
        match self {
            Self::Ok => 0,
            Self::NotSupported => 1,
            Self::BadState => 2,
            Self::NotAlive => 3,
            Self::Locked => 4,
            Self::Busy => 5,
            Self::NvmError => 6,
            Self::InvalidAddr => 7,
            Self::Timeout => 8,
            Self::Link => 9,
        }
    }

    /// Parses a wire byte into a status.
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Ok),
            1 => Some(Self::NotSupported),
            2 => Some(Self::BadState),
            3 => Some(Self::NotAlive),
            4 => Some(Self::Locked),
            5 => Some(Self::Busy),
            6 => Some(Self::NvmError),
            7 => Some(Self::InvalidAddr),
            8 => Some(Self::Timeout),
            9 => Some(Self::Link),
            _ => None,
        }
    }
}

/// Frame type byte of the session link.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionCmd {
    /// Chip-erase the target and enter programming mode.
    Begin,
    /// Program the carried data at the carried address.
    PageWrite,
    /// Read back flash at the carried address.
    PageRead,
    /// Leave programming mode and reset the target.
    End,
    /// Status reply to any command.
    Status,
    /// Read-back data reply to `PageRead`.
    PageData,
}

impl SessionCmd {
    /// Returns the wire byte for this command.
    #[must_use]
    pub const fn to_code(self) -> u8 {
        match self {
            Self::Begin => 1,
            Self::PageWrite => 2,
            Self::PageRead => 3,
            Self::End => 4,
            Self::Status => 5,
            Self::PageData => 6,
        }
    }

    /// Parses a wire byte into a command.
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            1 => Some(Self::Begin),
            2 => Some(Self::PageWrite),
            3 => Some(Self::PageRead),
            4 => Some(Self::End),
            5 => Some(Self::Status),
            6 => Some(Self::PageData),
            _ => None,
        }
    }
}

/// A decoded session command, borrowing page data from the frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command<'a> {
    /// Chip-erase the target and enter programming mode.
    Begin(SessionTarget),
    /// Program `data` at `addr`. Flash offsets must be even.
    PageWrite {
        /// Flash byte offset.
        addr: u16,
        /// Data to program.
        data: &'a [u8],
    },
    /// Read back `len` bytes at `addr`.
    #[cfg(feature = "page-read")]
    PageRead {
        /// Flash byte offset.
        addr: u16,
        /// Number of bytes to read.
        len: u8,
    },
    /// Leave programming mode and reset the target.
    End,
}

/// A session reply, borrowing read-back data from the caller's buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reply<'a> {
    /// The outcome of a command. `addr` is set for page commands so the
    /// master can match the reply to its request.
    Status {
        /// Outcome code.
        status: SessionStatus,
        /// The address the status refers to, for page commands.
        addr: Option<u16>,
    },
    /// Read-back data for `PageRead`.
    #[cfg(feature = "page-read")]
    PageData {
        /// Outcome code.
        status: SessionStatus,
        /// The address the data was read from.
        addr: u16,
        /// The read-back bytes.
        data: &'a [u8],
    },
    /// Keeps the lifetime parameter used when `page-read` is off. Never
    /// constructed.
    #[cfg(not(feature = "page-read"))]
    #[doc(hidden)]
    Unused(core::marker::PhantomData<&'a [u8]>),
}

/// Encodes a session command into `out` as a complete frame (command byte,
/// body, CRC-16), returning its length. The result is pre-COBS.
///
/// Returns `None` if `out` is too small or the page data is empty or
/// oversized.
#[must_use]
pub fn encode_command(cmd: Command<'_>, out: &mut [u8]) -> Option<usize> {
    let body_len = match cmd {
        Command::Begin(target) => {
            *out.first_mut()? = SessionCmd::Begin.to_code();
            let body = encode_begin(target);
            write_at(out, 1, &body)?;
            body.len()
        }
        Command::PageWrite { addr, data } => {
            if data.is_empty() || data.len() > PAGE_MAX {
                return None;
            }
            *out.first_mut()? = SessionCmd::PageWrite.to_code();
            write_addr_body(out, addr, data)?;
            2 + data.len()
        }
        #[cfg(feature = "page-read")]
        Command::PageRead { addr, len } => {
            *out.first_mut()? = SessionCmd::PageRead.to_code();
            let body = encode_read(addr, len);
            write_at(out, 1, &body)?;
            body.len()
        }
        Command::End => {
            *out.first_mut()? = SessionCmd::End.to_code();
            0
        }
    };
    finish_frame(out, body_len + 1)
}

/// Decodes a complete, COBS-decoded session command frame, checking its CRC.
///
/// Returns `None` if the CRC does not match, the command byte is unknown, or
/// the body is malformed.
#[must_use]
pub fn decode_command(frame: &[u8]) -> Option<Command<'_>> {
    let body = split_frame(frame)?;
    let (&code, rest) = body.split_first()?;
    match SessionCmd::from_code(code)? {
        SessionCmd::Begin => Some(Command::Begin(decode_begin(rest)?)),
        SessionCmd::PageWrite => {
            let (addr, data) = decode_write(rest)?;
            Some(Command::PageWrite { addr, data })
        }
        #[cfg(feature = "page-read")]
        SessionCmd::PageRead => {
            let (addr, len) = decode_read(rest)?;
            Some(Command::PageRead { addr, len })
        }
        SessionCmd::End if rest.is_empty() => Some(Command::End),
        _ => None,
    }
}

/// Encodes a session reply into `out` as a complete frame, returning its
/// length. The result is pre-COBS.
///
/// Returns `None` if `out` is too small or the reply carries oversized data.
#[must_use]
pub fn encode_reply(reply: Reply<'_>, out: &mut [u8]) -> Option<usize> {
    let body_len = match reply {
        Reply::Status { status, addr } => {
            *out.first_mut()? = SessionCmd::Status.to_code();
            let body = encode_page_status(status, addr.unwrap_or(0));
            let len = addr.map_or(1, |_| body.len());
            write_at(out, 1, body.get(..len).unwrap_or(&[]))?;
            len
        }
        #[cfg(feature = "page-read")]
        Reply::PageData { status, addr, data } => {
            if data.len() > PAGE_MAX {
                return None;
            }
            *out.first_mut()? = SessionCmd::PageData.to_code();
            let head = encode_page_status(status, addr);
            write_at(out, 1, &head)?;
            write_at(out, 1 + head.len(), data)?;
            head.len() + data.len()
        }
        #[cfg(not(feature = "page-read"))]
        Reply::Unused(_) => return None,
    };
    finish_frame(out, body_len + 1)
}

/// Decodes a complete, COBS-decoded session reply frame, checking its CRC.
///
/// Returns `None` if the CRC does not match, the command byte is unknown, or
/// the body is malformed.
#[must_use]
pub fn decode_reply(frame: &[u8]) -> Option<Reply<'_>> {
    let body = split_frame(frame)?;
    let (&code, rest) = body.split_first()?;
    match SessionCmd::from_code(code)? {
        SessionCmd::Status => {
            let (status, rest) = rest.split_first_chunk::<1>()?;
            let status = SessionStatus::from_code(status[0])?;
            let addr = if rest.is_empty() {
                None
            } else {
                let (addr_bytes, _) = rest.split_first_chunk::<2>()?;
                Some(u16::from_le_bytes(*addr_bytes))
            };
            Some(Reply::Status { status, addr })
        }
        #[cfg(feature = "page-read")]
        SessionCmd::PageData => {
            let (status, addr, data) = decode_page_data(rest)?;
            Some(Reply::PageData { status, addr, data })
        }
        _ => None,
    }
}

fn split_frame(frame: &[u8]) -> Option<&[u8]> {
    let split = frame.len().checked_sub(CRC_LEN)?;
    let (body, crc_bytes) = frame.split_at(split);
    let expected = u16::from_le_bytes(crc_bytes.try_into().ok()?);
    if crc::checksum16(body) != expected {
        return None;
    }
    Some(body)
}

fn finish_frame(out: &mut [u8], len: usize) -> Option<usize> {
    let covered = out.get(..len)?;
    let crc = crc::checksum16(covered);
    let tail = out.get_mut(len..len + CRC_LEN)?;
    tail.copy_from_slice(&crc.to_le_bytes());
    Some(len + CRC_LEN)
}

fn write_at(out: &mut [u8], at: usize, bytes: &[u8]) -> Option<()> {
    let slot = out.get_mut(at..at + bytes.len())?;
    // A byte loop instead of `copy_from_slice`: the variable-length copy
    // would link the generic `memcpy`, which costs more flash.
    for (dst, src) in slot.iter_mut().zip(bytes) {
        *dst = *src;
    }
    Some(())
}

fn write_addr_body(out: &mut [u8], addr: u16, data: &[u8]) -> Option<()> {
    let head = out.get_mut(1..3)?;
    head.copy_from_slice(&addr.to_le_bytes());
    write_at(out, 3, data)
}

/// Encodes the payload of a `ProgSessionBegin` command.
#[must_use]
pub const fn encode_begin(target: SessionTarget) -> [u8; 1] {
    [target.to_code()]
}

/// Decodes the payload of a `ProgSessionBegin` command.
#[must_use]
pub fn decode_begin(payload: &[u8]) -> Option<SessionTarget> {
    SessionTarget::from_code(*payload.first()?)
}

/// Encodes a `ProgPageWrite` payload. `data` must not be empty nor longer
/// than [`PAGE_MAX`].
#[must_use]
pub fn encode_write<'a>(addr: u16, data: &[u8], out: &'a mut [u8]) -> Option<&'a [u8]> {
    let len = 2 + data.len();
    if data.is_empty() || data.len() > PAGE_MAX || out.len() < len {
        return None;
    }
    let (head, rest) = out.split_at_mut(2);
    head.copy_from_slice(&addr.to_le_bytes());
    // A byte loop instead of `copy_from_slice`: the variable-length copy
    // would link the generic `memcpy`, which costs more flash.
    for (dst, src) in rest.iter_mut().zip(data) {
        *dst = *src;
    }
    out.get(..len)
}

/// Decodes a `ProgPageWrite` payload. The data slice borrows from `payload`.
#[must_use]
pub fn decode_write(payload: &[u8]) -> Option<(u16, &[u8])> {
    let (addr_bytes, data) = payload.split_first_chunk::<2>()?;
    let addr = u16::from_le_bytes(*addr_bytes);
    if data.is_empty() || data.len() > PAGE_MAX {
        return None;
    }
    Some((addr, data))
}

/// Encodes the payload of a `PageRead` command.
#[cfg(feature = "page-read")]
#[must_use]
pub const fn encode_read(addr: u16, len: u8) -> [u8; 3] {
    let [a0, a1] = addr.to_le_bytes();
    [a0, a1, len]
}

/// Decodes the payload of a `PageRead` command into the address and the
/// requested length.
#[cfg(feature = "page-read")]
#[must_use]
pub fn decode_read(payload: &[u8]) -> Option<(u16, u8)> {
    let (addr_bytes, rest) = payload.split_first_chunk::<2>()?;
    Some((u16::from_le_bytes(*addr_bytes), *rest.first()?))
}

/// Encodes a `ProgSessionStatus` reply payload: status byte, then the 2
/// address bytes it refers to.
#[must_use]
pub const fn encode_page_status(status: SessionStatus, addr: u16) -> [u8; 3] {
    let [a0, a1] = addr.to_le_bytes();
    [status.to_code(), a0, a1]
}

/// Decodes a `ProgSessionStatus` reply payload.
#[must_use]
pub fn decode_page_status(payload: &[u8]) -> Option<(SessionStatus, u16)> {
    let (status, rest) = payload.split_first_chunk::<1>()?;
    let (addr_bytes, _) = rest.split_first_chunk::<2>()?;
    Some((
        SessionStatus::from_code(status[0])?,
        u16::from_le_bytes(*addr_bytes),
    ))
}

/// Encodes the payload of a `ProgPageData` reply into `out`.
///
/// An error reply carries no data, which [`decode_page_data`] reports as an
/// empty slice. Returns `None` if `data` is longer than [`PAGE_MAX`] or `out`
/// is too small.
#[must_use]
pub fn encode_page_data<'a>(
    status: SessionStatus,
    addr: u16,
    data: &[u8],
    out: &'a mut [u8],
) -> Option<&'a [u8]> {
    let len = 3 + data.len();
    if data.len() > PAGE_MAX || out.len() < len {
        return None;
    }
    let (head, rest) = out.split_at_mut(3);
    head.copy_from_slice(&encode_page_status(status, addr));
    for (dst, src) in rest.iter_mut().zip(data) {
        *dst = *src;
    }
    out.get(..len)
}

/// Decodes the payload of a `ProgPageData` reply. The data slice borrows from
/// `payload`.
#[must_use]
pub fn decode_page_data(payload: &[u8]) -> Option<(SessionStatus, u16, &[u8])> {
    let (status, addr) = decode_page_status(payload)?;
    let data = payload.get(3..)?;
    if data.len() > PAGE_MAX {
        return None;
    }
    Some((status, addr, data))
}

#[cfg(test)]
mod tests {
    use super::{
        Command, PAGE_MAX, Reply, SessionCmd, SessionStatus, SessionTarget, decode_begin,
        decode_command, decode_page_data, decode_page_status, decode_reply, decode_write,
        encode_begin, encode_command, encode_page_data, encode_page_status, encode_reply,
        encode_write,
    };
    #[cfg(feature = "page-read")]
    use super::{decode_read, encode_read};

    #[test]
    fn target_roundtrips() {
        for target in [SessionTarget::Cellagent, SessionTarget::Cellcore] {
            assert_eq!(
                SessionTarget::from_code(target.to_code()),
                Some(target),
                "target must roundtrip"
            );
        }
        assert_eq!(SessionTarget::from_code(2), None);
    }

    #[test]
    fn status_roundtrips() {
        let all = [
            SessionStatus::Ok,
            SessionStatus::NotSupported,
            SessionStatus::BadState,
            SessionStatus::NotAlive,
            SessionStatus::Locked,
            SessionStatus::Busy,
            SessionStatus::NvmError,
            SessionStatus::InvalidAddr,
            SessionStatus::Timeout,
            SessionStatus::Link,
        ];
        for status in all {
            assert_eq!(
                SessionStatus::from_code(status.to_code()),
                Some(status),
                "status must roundtrip"
            );
        }
        assert_eq!(SessionStatus::from_code(10), None);
    }

    #[test]
    fn begin_payload_roundtrips() {
        let payload = encode_begin(SessionTarget::Cellagent);
        assert_eq!(decode_begin(&payload), Some(SessionTarget::Cellagent));
    }

    #[test]
    fn write_payload_roundtrips() {
        let mut out = [0u8; 2 + PAGE_MAX];
        let data = [0xA5u8; PAGE_MAX];
        let payload = encode_write(0x1234, &data, &mut out).expect("fits");
        let (addr, decoded) = decode_write(payload).expect("decodes");
        assert_eq!(addr, 0x1234);
        assert_eq!(decoded, &data);
    }

    #[test]
    fn write_payload_rejects_empty_and_oversized() {
        assert!(decode_write(&[0x00, 0x10]).is_none(), "empty data");
        let oversized = [0u8; 2 + PAGE_MAX + 1];
        assert!(decode_write(&oversized).is_none(), "oversized data");
        let mut out = [0u8; 2 + PAGE_MAX];
        assert!(encode_write(0, &[], &mut out).is_none(), "empty data");
    }

    #[test]
    fn write_payload_short_page_uses_prefix_of_buffer() {
        let mut out = [0xFFu8; 2 + PAGE_MAX];
        let data = [1, 2, 3];
        let payload = encode_write(0x0200, &data, &mut out).expect("fits");
        let (addr, decoded) = decode_write(payload).expect("decodes");
        assert_eq!(addr, 0x0200);
        assert_eq!(decoded, &data);
    }

    #[cfg(feature = "page-read")]
    #[test]
    fn read_payload_roundtrips() {
        let page_max = u8::try_from(PAGE_MAX).expect("PAGE_MAX fits u8");
        let payload = encode_read(0xBEEF, page_max);
        assert_eq!(decode_read(&payload), Some((0xBEEF, page_max)));
    }

    #[test]
    fn page_status_roundtrips() {
        let payload = encode_page_status(SessionStatus::Busy, 0x0102);
        assert_eq!(
            decode_page_status(&payload),
            Some((SessionStatus::Busy, 0x0102))
        );
    }

    #[test]
    fn page_data_roundtrips() {
        let mut out = [0u8; 3 + PAGE_MAX];
        let data = core::array::from_fn::<u8, PAGE_MAX, _>(|i| u8::try_from(i).unwrap());
        let payload = encode_page_data(SessionStatus::Ok, 0x0400, &data, &mut out).expect("fits");
        let (status, addr, decoded) = decode_page_data(payload).expect("decodes");
        assert_eq!(status, SessionStatus::Ok);
        assert_eq!(addr, 0x0400);
        assert_eq!(decoded, &data);
    }

    #[test]
    fn command_frames_roundtrip() {
        let mut out = [0u8; 1 + 2 + PAGE_MAX + 2];
        let commands = [
            Command::Begin(SessionTarget::Cellagent),
            Command::Begin(SessionTarget::Cellcore),
            Command::PageWrite {
                addr: 0x1234,
                data: &[0xA5; PAGE_MAX],
            },
            Command::PageWrite {
                addr: 0x0100,
                data: &[1, 2, 3],
            },
            #[cfg(feature = "page-read")]
            Command::PageRead {
                addr: 0xBEEF,
                len: 7,
            },
            Command::End,
        ];
        for cmd in commands {
            let n = encode_command(cmd, &mut out).expect("fits");
            assert_eq!(decode_command(&out[..n]), Some(cmd), "{cmd:?}");
        }
    }

    #[test]
    fn reply_frames_roundtrip() {
        let mut out = [0u8; 1 + 3 + PAGE_MAX + 2];
        let replies = [
            Reply::Status {
                status: SessionStatus::Ok,
                addr: None,
            },
            Reply::Status {
                status: SessionStatus::Busy,
                addr: Some(0x0400),
            },
            #[cfg(feature = "page-read")]
            Reply::PageData {
                status: SessionStatus::Ok,
                addr: 0x0400,
                data: &[0x5A; PAGE_MAX],
            },
        ];
        for reply in replies {
            let n = encode_reply(reply, &mut out).expect("fits");
            assert_eq!(decode_reply(&out[..n]), Some(reply), "{reply:?}");
        }
    }

    #[test]
    fn corrupt_frame_fails_the_crc() {
        let mut out = [0u8; 32];
        let n = encode_command(
            Command::PageWrite {
                addr: 0x0200,
                data: &[1, 2, 3, 4],
            },
            &mut out,
        )
        .expect("fits");
        out[0] ^= 0x01;
        assert_eq!(decode_command(&out[..n]), None);
        out[0] ^= 0x01;
        assert!(decode_command(&out[..n]).is_some());
    }

    #[test]
    fn encode_rejects_malformed_commands() {
        let mut out = [0u8; 1 + 2 + PAGE_MAX + 2];
        assert!(
            encode_command(Command::PageWrite { addr: 0, data: &[] }, &mut out).is_none(),
            "empty page data"
        );
        let oversized = [0u8; PAGE_MAX + 1];
        assert!(
            encode_command(
                Command::PageWrite {
                    addr: 0,
                    data: &oversized,
                },
                &mut out
            )
            .is_none(),
            "oversized page data"
        );
        let mut small = [0u8; 2];
        assert!(
            encode_command(Command::End, &mut small).is_none(),
            "no room"
        );
    }

    #[test]
    fn end_with_a_body_is_rejected() {
        let body = [SessionCmd::End.to_code(), 0x00];
        let crc = crc::checksum16(&body);
        let mut wire = [0u8; 4];
        wire[..2].copy_from_slice(&body);
        wire[2..].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(decode_command(&wire), None);
    }

    #[test]
    fn page_data_error_reply_is_short() {
        let mut out = [0u8; 3 + PAGE_MAX];
        let payload = encode_page_data(SessionStatus::Locked, 0x0400, &[], &mut out).expect("fits");
        let (status, addr, decoded) = decode_page_data(payload).expect("decodes");
        assert_eq!(status, SessionStatus::Locked);
        assert_eq!(addr, 0x0400);
        assert!(decoded.is_empty());
    }
}
