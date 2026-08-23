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

use core::mem::size_of_val;

use zerocopy::byteorder::little_endian::U16;
use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned};

/// Maximum data bytes carried by one page command or reply.
pub const PAGE_MAX: usize = 64;

/// Worst-case COBS-encoded size of the largest command (`PageWrite`).
pub const MAX_COMMAND_WIRE: usize = crate::max_encoded_len(1 + 2 + PAGE_MAX + CRC_LEN);

/// Worst-case COBS-encoded size of the largest reply (`PageData`).
pub const MAX_REPLY_WIRE: usize = crate::max_encoded_len(1 + 3 + PAGE_MAX + CRC_LEN);

/// Length of the frame CRC in bytes.
const CRC_LEN: usize = 2;

/// Wire body of a `PageRead` command.
#[cfg(feature = "page-read")]
#[derive(FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C)]
struct PageReadBody {
    addr: U16,
    len: u8,
}

/// Wire body of a `ProgSessionStatus` reply: status byte, then the address
/// it refers to.
#[derive(FromBytes, IntoBytes, Immutable, KnownLayout, Unaligned)]
#[repr(C)]
struct StatusBody {
    status: u8,
    addr: U16,
}

/// Which target a session should program.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionTarget {
    /// The `cellagent` balancer MCU, reached over UPDI mux channel 3.
    Cellagent,
    /// The `cellcore` MCU over mux channel 1. Reserved: this programmer
    /// answers [`SessionStatus::NotSupported`].
    Cellcore,
    /// The programmer itself. `Begin` does not erase anything: the servant
    /// stages the payload, verifies it, and rewrites its own flash after
    /// reset (see the `cellprog` firmware's walker).
    CellprogSelf,
}

impl SessionTarget {
    /// Returns the wire byte for this target.
    #[must_use]
    pub const fn to_code(self) -> u8 {
        match self {
            Self::Cellagent => 0,
            Self::Cellcore => 1,
            Self::CellprogSelf => 2,
        }
    }

    /// Parses a wire byte into a target.
    #[must_use]
    pub const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Cellagent),
            1 => Some(Self::Cellcore),
            2 => Some(Self::CellprogSelf),
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
enum SessionCmd {
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
    const fn to_code(self) -> u8 {
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
    const fn from_code(code: u8) -> Option<Self> {
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

impl<'a> Command<'a> {
    /// Encodes the command into `out` as a complete frame (command byte,
    /// body, CRC-16), returning its length. The result is pre-COBS.
    ///
    /// Returns `None` if `out` is too small or the page data is empty or
    /// oversized.
    #[must_use]
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let body_len = match *self {
            Self::Begin(target) => {
                *out.first_mut()? = SessionCmd::Begin.to_code();
                let body = [target.to_code()];
                write_at(out, 1, &body)?;
                body.len()
            }
            Self::PageWrite { addr, data } => {
                if data.is_empty() || data.len() > PAGE_MAX {
                    return None;
                }
                *out.first_mut()? = SessionCmd::PageWrite.to_code();
                write_addr_body(out, addr, data)?;
                2 + data.len()
            }
            #[cfg(feature = "page-read")]
            Self::PageRead { addr, len } => {
                *out.first_mut()? = SessionCmd::PageRead.to_code();
                let body = PageReadBody {
                    addr: U16::new(addr),
                    len,
                };
                write_body(out, 1, &body)?;
                size_of_val(&body)
            }
            Self::End => {
                *out.first_mut()? = SessionCmd::End.to_code();
                0
            }
        };
        finish_frame(out, body_len + 1)
    }

    /// Decodes a complete, COBS-decoded command frame, checking its CRC.
    ///
    /// Returns `None` if the CRC does not match, the command byte is unknown,
    /// or the body is malformed.
    #[must_use]
    pub fn decode(frame: &'a [u8]) -> Option<Self> {
        let body = split_frame(frame)?;
        let (&code, rest) = body.split_first()?;
        match SessionCmd::from_code(code)? {
            SessionCmd::Begin => Some(Self::Begin(SessionTarget::from_code(*rest.first()?)?)),
            SessionCmd::PageWrite => {
                let (addr, data) = U16::ref_from_prefix(rest).ok()?;
                if data.is_empty() || data.len() > PAGE_MAX {
                    return None;
                }
                Some(Self::PageWrite {
                    addr: addr.get(),
                    data,
                })
            }
            #[cfg(feature = "page-read")]
            SessionCmd::PageRead => {
                let (body, _) = PageReadBody::ref_from_prefix(rest).ok()?;
                Some(Self::PageRead {
                    addr: body.addr.get(),
                    len: body.len,
                })
            }
            SessionCmd::End if rest.is_empty() => Some(Self::End),
            _ => None,
        }
    }
}

impl<'a> Reply<'a> {
    /// Encodes the reply into `out` as a complete frame, returning its
    /// length. The result is pre-COBS.
    ///
    /// Returns `None` if `out` is too small or the reply carries oversized
    /// data.
    #[must_use]
    pub fn encode(&self, out: &mut [u8]) -> Option<usize> {
        let body_len = match *self {
            Self::Status { status, addr } => {
                *out.first_mut()? = SessionCmd::Status.to_code();
                if let Some(addr) = addr {
                    let body = StatusBody {
                        status: status.to_code(),
                        addr: U16::new(addr),
                    };
                    write_body(out, 1, &body)?;
                    size_of_val(&body)
                } else {
                    *out.get_mut(1)? = status.to_code();
                    1
                }
            }
            #[cfg(feature = "page-read")]
            Self::PageData { status, addr, data } => {
                if data.len() > PAGE_MAX {
                    return None;
                }
                *out.first_mut()? = SessionCmd::PageData.to_code();
                let head = StatusBody {
                    status: status.to_code(),
                    addr: U16::new(addr),
                };
                write_body(out, 1, &head)?;
                let head_len = size_of_val(&head);
                write_at(out, 1 + head_len, data)?;
                head_len + data.len()
            }
            #[cfg(not(feature = "page-read"))]
            Self::Unused(_) => return None,
        };
        finish_frame(out, body_len + 1)
    }

    /// Decodes a complete, COBS-decoded reply frame, checking its CRC.
    ///
    /// Returns `None` if the CRC does not match, the command byte is unknown,
    /// or the body is malformed.
    #[must_use]
    pub fn decode(frame: &'a [u8]) -> Option<Self> {
        let body = split_frame(frame)?;
        let (&code, rest) = body.split_first()?;
        match SessionCmd::from_code(code)? {
            SessionCmd::Status => {
                let (&status, rest) = rest.split_first()?;
                let status = SessionStatus::from_code(status)?;
                let addr = if rest.is_empty() {
                    None
                } else {
                    Some(U16::ref_from_prefix(rest).ok()?.0.get())
                };
                Some(Self::Status { status, addr })
            }
            #[cfg(feature = "page-read")]
            SessionCmd::PageData => {
                let (head, data) = StatusBody::ref_from_prefix(rest).ok()?;
                let status = SessionStatus::from_code(head.status)?;
                if data.len() > PAGE_MAX {
                    return None;
                }
                Some(Self::PageData {
                    status,
                    addr: head.addr.get(),
                    data,
                })
            }
            _ => None,
        }
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

/// Writes a fixed-layout body at `at`. Unlike [`write_at`], the constant
/// length lets `copy_from_slice` inline instead of linking `memcpy`.
fn write_body<T>(out: &mut [u8], at: usize, body: &T) -> Option<()>
where
    T: IntoBytes + Immutable,
{
    let bytes = body.as_bytes();
    let slot = out.get_mut(at..at + bytes.len())?;
    slot.copy_from_slice(bytes);
    Some(())
}

fn write_addr_body(out: &mut [u8], addr: u16, data: &[u8]) -> Option<()> {
    let head = out.get_mut(1..3)?;
    head.copy_from_slice(U16::new(addr).as_bytes());
    write_at(out, 3, data)
}

#[cfg(test)]
mod tests {
    use super::{Command, PAGE_MAX, Reply, SessionCmd, SessionStatus, SessionTarget};

    /// Builds `body` plus its CRC-16 in `out`, returning the frame length.
    fn frame_with_crc(body: &[u8], out: &mut [u8]) -> usize {
        let n = body.len();
        out[..n].copy_from_slice(body);
        out[n..n + 2].copy_from_slice(&crc::checksum16(body).to_le_bytes());
        n + 2
    }

    /// Asserts that `frame` is exactly `body` plus its CRC-16.
    fn assert_frame(frame: &[u8], body: &[u8]) {
        assert_eq!(frame.len(), body.len() + 2);
        let n = body.len();
        assert_eq!(&frame[..n], body);
        let crc = crc::checksum16(body);
        assert_eq!(&frame[n..], &crc.to_le_bytes());
    }

    #[test]
    fn target_roundtrips() {
        for target in [
            SessionTarget::Cellagent,
            SessionTarget::Cellcore,
            SessionTarget::CellprogSelf,
        ] {
            assert_eq!(
                SessionTarget::from_code(target.to_code()),
                Some(target),
                "target must roundtrip"
            );
        }
        assert_eq!(SessionTarget::from_code(3), None);
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
    fn begin_frame_wire_bytes_are_frozen() {
        let mut out = [0u8; 8];
        let n = Command::Begin(SessionTarget::Cellagent)
            .encode(&mut out)
            .expect("fits");
        assert_frame(
            &out[..n],
            &[
                SessionCmd::Begin.to_code(),
                SessionTarget::Cellagent.to_code(),
            ],
        );
    }

    #[test]
    fn command_frames_roundtrip() {
        let mut out = [0u8; 1 + 2 + PAGE_MAX + 2];
        let commands = [
            Command::Begin(SessionTarget::Cellagent),
            Command::Begin(SessionTarget::Cellcore),
            Command::Begin(SessionTarget::CellprogSelf),
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
            let n = cmd.encode(&mut out).expect("fits");
            assert_eq!(Command::decode(&out[..n]), Some(cmd), "{cmd:?}");
        }
    }

    #[test]
    fn decode_rejects_malformed_page_write_bodies() {
        let mut buf = [0u8; 6 + PAGE_MAX];
        let n = frame_with_crc(&[SessionCmd::PageWrite.to_code(), 0x34, 0x12], &mut buf);
        assert!(Command::decode(&buf[..n]).is_none(), "empty data");
        let mut body = [0u8; 4 + PAGE_MAX];
        body[0] = SessionCmd::PageWrite.to_code();
        let n = frame_with_crc(&body, &mut buf);
        assert!(Command::decode(&buf[..n]).is_none(), "oversized data");
    }

    #[cfg(feature = "page-read")]
    #[test]
    fn page_read_wire_bytes_are_frozen() {
        let mut out = [0u8; 8];
        let n = Command::PageRead {
            addr: 0xBEEF,
            len: 7,
        }
        .encode(&mut out)
        .expect("fits");
        assert_frame(&out[..n], &[SessionCmd::PageRead.to_code(), 0xEF, 0xBE, 7]);
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
            let n = reply.encode(&mut out).expect("fits");
            assert_eq!(Reply::decode(&out[..n]), Some(reply), "{reply:?}");
        }
    }

    #[test]
    fn status_reply_wire_bytes_are_frozen() {
        let mut out = [0u8; 8];
        // Without an address the body is a single status byte.
        let n = Reply::Status {
            status: SessionStatus::Ok,
            addr: None,
        }
        .encode(&mut out)
        .expect("fits");
        assert_frame(
            &out[..n],
            &[SessionCmd::Status.to_code(), SessionStatus::Ok.to_code()],
        );
        // With an address the body appends it in little-endian order.
        let n = Reply::Status {
            status: SessionStatus::Busy,
            addr: Some(0x0402),
        }
        .encode(&mut out)
        .expect("fits");
        assert_frame(
            &out[..n],
            &[
                SessionCmd::Status.to_code(),
                SessionStatus::Busy.to_code(),
                0x02,
                0x04,
            ],
        );
    }

    #[cfg(feature = "page-read")]
    #[test]
    fn page_data_head_wire_bytes_are_frozen() {
        let mut out = [0u8; 16];
        let n = Reply::PageData {
            status: SessionStatus::Busy,
            addr: 0x0204,
            data: &[1, 2, 3],
        }
        .encode(&mut out)
        .expect("fits");
        assert_frame(
            &out[..n],
            &[
                SessionCmd::PageData.to_code(),
                SessionStatus::Busy.to_code(),
                0x04,
                0x02,
                1,
                2,
                3,
            ],
        );
    }

    #[test]
    fn corrupt_frame_fails_the_crc() {
        let mut out = [0u8; 32];
        let n = Command::PageWrite {
            addr: 0x0200,
            data: &[1, 2, 3, 4],
        }
        .encode(&mut out)
        .expect("fits");
        out[0] ^= 0x01;
        assert_eq!(Command::decode(&out[..n]), None);
        out[0] ^= 0x01;
        assert!(Command::decode(&out[..n]).is_some());
    }

    #[test]
    fn encode_rejects_malformed_commands() {
        let mut out = [0u8; 1 + 2 + PAGE_MAX + 2];
        assert!(
            Command::PageWrite { addr: 0, data: &[] }
                .encode(&mut out)
                .is_none(),
            "empty page data"
        );
        let oversized = [0u8; PAGE_MAX + 1];
        assert!(
            Command::PageWrite {
                addr: 0,
                data: &oversized
            }
            .encode(&mut out)
            .is_none(),
            "oversized page data"
        );
        let mut small = [0u8; 2];
        assert!(Command::End.encode(&mut small).is_none(), "no room");
    }

    #[test]
    fn end_with_a_body_is_rejected() {
        let mut buf = [0u8; 4];
        let n = frame_with_crc(&[SessionCmd::End.to_code(), 0x00], &mut buf);
        assert_eq!(Command::decode(&buf[..n]), None);
    }

    #[cfg(feature = "page-read")]
    #[test]
    fn page_data_error_reply_is_short() {
        let mut out = [0u8; 1 + 3 + PAGE_MAX + 2];
        let reply = Reply::PageData {
            status: SessionStatus::Locked,
            addr: 0x0400,
            data: &[],
        };
        let n = reply.encode(&mut out).expect("fits");
        assert_eq!(Reply::decode(&out[..n]), Some(reply));
        assert_eq!(n, 1 + 3 + 2);
    }
}
