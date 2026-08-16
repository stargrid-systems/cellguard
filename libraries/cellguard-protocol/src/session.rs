//! The transactional programming session over the local `UART_PROG` link.
//!
//! The `cellprog` MCU has one USART reached through an analog mux: channel 0
//! is the UART to the cellcore, the other channels are UPDI lines. While the
//! programmer talks UPDI its UART path to the cellcore is physically
//! disconnected, so a transparent byte pipe is impossible. Instead the
//! programmer services one command per transaction: receive a complete
//! command on channel 0, switch the mux to the target, run one UPDI
//! operation, switch back, reply.
//!
//! A session is:
//!
//! 1. [`Kind::ProgSessionBegin`](crate::Kind::ProgSessionBegin): the programmer
//!    chip-erases the target and resets it into programming mode. Erase first
//!    means every page write lands on blank flash; a retry of `Begin` simply
//!    restarts the session from a blank chip.
//! 2. `ProgPageWrite` x N: the master streams the image, at most [`PAGE_MAX`]
//!    data bytes per command. Addresses are byte offsets into the target's
//!    flash (0-based; the programmer maps them into the target's data space).
//!    Writes are idempotent: a re-sent identical command programs the same
//!    bytes.
//! 3. `ProgPageRead` x N: the master reads flash back to verify against its own
//!    copy of the image.
//! 4. [`Kind::ProgSessionEnd`](crate::Kind::ProgSessionEnd): the programmer
//!    resets the target out of programming mode.
//!
//! Page commands before a successful `Begin` are rejected with
//! [`SessionStatus::BadState`]: writing un-erased flash corrupts it.
//!
//! Exactly one command may be in flight. Commands sent while the mux is on a
//! UPDI channel are electrically lost, never buffered.

/// Maximum data bytes carried by one page command or reply.
pub const PAGE_MAX: usize = 64;

/// Worst-case COBS-encoded size of the largest command (`ProgPageWrite`).
pub const MAX_COMMAND_WIRE: usize =
    crate::max_encoded_len(crate::HEADER_LEN + 2 + PAGE_MAX + crate::PAYLOAD_CRC_LEN);

/// Worst-case COBS-encoded size of the largest reply (`ProgPageData`).
pub const MAX_REPLY_WIRE: usize =
    crate::max_encoded_len(crate::HEADER_LEN + 3 + PAGE_MAX + crate::PAYLOAD_CRC_LEN);

/// Which target a session should program.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionTarget {
    /// The `cellagent` balancer MCU, reached over UPDI mux channel 3.
    Cellagent,
    /// The `cellcore` MCU over mux channel 1. Reserved for a future
    /// programmer build. The current servant firmware does not support it and
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

/// Encodes the payload of a `ProgPageWrite` command into `out`, returning the
/// encoded slice. `data` must not be empty nor longer than [`PAGE_MAX`].
#[must_use]
pub fn encode_write<'a>(addr: u16, data: &[u8], out: &'a mut [u8]) -> Option<&'a [u8]> {
    let len = 2 + data.len();
    if data.is_empty() || data.len() > PAGE_MAX || out.len() < len {
        return None;
    }
    let (head, rest) = out.split_at_mut(2);
    head.copy_from_slice(&addr.to_le_bytes());
    rest.get_mut(..data.len())?.copy_from_slice(data);
    out.get(..len)
}

/// Decodes the payload of a `ProgPageWrite` command into the address and the
/// data slice, which borrows from `payload`.
#[must_use]
pub fn decode_write(payload: &[u8]) -> Option<(u16, &[u8])> {
    let (addr_bytes, data) = payload.split_first_chunk::<2>()?;
    let addr = u16::from_le_bytes(*addr_bytes);
    if data.is_empty() || data.len() > PAGE_MAX {
        return None;
    }
    Some((addr, data))
}

/// Encodes the payload of a `ProgPageRead` command.
#[must_use]
pub const fn encode_read(addr: u16, len: u8) -> [u8; 3] {
    let [a0, a1] = addr.to_le_bytes();
    [a0, a1, len]
}

/// Decodes the payload of a `ProgPageRead` command into the address and the
/// requested length.
#[must_use]
pub fn decode_read(payload: &[u8]) -> Option<(u16, u8)> {
    let (addr_bytes, rest) = payload.split_first_chunk::<2>()?;
    Some((u16::from_le_bytes(*addr_bytes), *rest.first()?))
}

/// Encodes the payload of a `ProgSessionStatus` reply to a page command: the
/// status plus the address it refers to, so the master can match the reply to
/// its command.
#[must_use]
pub const fn encode_page_status(status: SessionStatus, addr: u16) -> [u8; 3] {
    let [a0, a1] = addr.to_le_bytes();
    [status.to_code(), a0, a1]
}

/// Decodes the payload of a `ProgSessionStatus` reply to a page command.
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
/// On success `data` holds the read-back bytes. An error reply carries no
/// data, which [`decode_page_data`] reports as an empty slice. Returns `None`
/// if `data` is longer than [`PAGE_MAX`] or `out` is too small.
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
    rest.get_mut(..data.len())?.copy_from_slice(data);
    out.get(..len)
}

/// Decodes the payload of a `ProgPageData` reply into the status, the address
/// it refers to, and the data slice, which borrows from `payload`.
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
        PAGE_MAX, SessionStatus, SessionTarget, decode_begin, decode_page_data, decode_page_status,
        decode_read, decode_write, encode_begin, encode_page_data, encode_page_status, encode_read,
        encode_write,
    };

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
    fn page_data_error_reply_is_short() {
        let mut out = [0u8; 3 + PAGE_MAX];
        let payload = encode_page_data(SessionStatus::Locked, 0x0400, &[], &mut out).expect("fits");
        let (status, addr, decoded) = decode_page_data(payload).expect("decodes");
        assert_eq!(status, SessionStatus::Locked);
        assert_eq!(addr, 0x0400);
        assert!(decoded.is_empty());
    }
}
