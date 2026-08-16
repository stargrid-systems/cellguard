//! The servant-side programming session handler.
//!
//! [`SessionHandler`] is the `cellprog` MCU's side of the transactional
//! programming protocol (see `cellguard_protocol::session`). It decodes one
//! command from the UART link, runs it against a `TinyProgrammer`, and
//! produces the raw reply frame ready to COBS-encode onto the wire.
//!
//! The handler does not own the USART or the mux. The firmware switches the
//! link between UART mode (decode and reply) and UPDI mode (execute) around
//! each command, so [`SessionHandler::decode`] and
//! [`SessionHandler::execute`] are separate calls.
//!
//! A command may only be decoded while the link is in UART mode, and a
//! returned [`Command`] must be executed before the next is decoded.

use cellguard_protocol::{
    Decoder, Kind, Packet, SessionStatus, SessionTarget, decode_begin, decode_read, decode_write,
    encode_page_status,
};
use updi::{ProgError, TinyProgrammer, UpdiLink};

/// Decoded size of the largest command frame (`ProgPageWrite`).
pub const MAX_COMMAND_FRAME: usize = cellguard_protocol::HEADER_LEN
    + 2
    + cellguard_protocol::PAGE_MAX
    + cellguard_protocol::PAYLOAD_CRC_LEN;

/// Decoded size of the largest reply frame (`ProgPageData`).
pub const MAX_REPLY_FRAME: usize = cellguard_protocol::HEADER_LEN
    + 3
    + cellguard_protocol::PAGE_MAX
    + cellguard_protocol::PAYLOAD_CRC_LEN;

const _: () = assert!(
    MAX_COMMAND_FRAME >= 3 + cellguard_protocol::PAGE_MAX,
    "rx must double as the page-read staging buffer"
);

/// A decoded session command. Page data lives in the handler, not here:
/// [`SessionHandler::execute`] reads it back.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    /// Chip-erase the target and enter programming mode.
    Begin(SessionTarget),
    /// Program the staged data at `addr`. Flash offsets must be even.
    PageWrite {
        /// Flash byte offset.
        addr: u16,
        /// Data length; the bytes live in the handler.
        len: usize,
    },
    /// Read back `len` bytes at `addr`.
    PageRead {
        /// Flash byte offset.
        addr: u16,
        /// Number of bytes to read.
        len: u8,
    },
    /// Leave programming mode and reset the target.
    End,
}

/// Maps a [`ProgError`] onto its wire status.
const fn status_of<E>(err: &ProgError<E>) -> SessionStatus {
    match err {
        ProgError::Updi(_) => SessionStatus::Link,
        ProgError::NotAlive => SessionStatus::NotAlive,
        ProgError::KeyRejected | ProgError::Locked => SessionStatus::Locked,
        ProgError::EnterTimeout | ProgError::EraseTimeout => SessionStatus::Timeout,
        ProgError::InvalidOffset => SessionStatus::InvalidAddr,
        ProgError::Busy => SessionStatus::Busy,
        ProgError::NvmError => SessionStatus::NvmError,
    }
}

/// The servant-side session state machine.
///
/// Owns the decode buffer, the reply buffer, and the one bit of session
/// state. See the [module](self) docs for the firmware calling pattern.
pub struct SessionHandler {
    id: u8,
    decoder: Decoder,
    in_session: bool,
    /// Decoded command frames; reused as the page-read staging buffer.
    rx: [u8; MAX_COMMAND_FRAME],
    /// Page-write data and the raw reply frame; never both at once.
    tx: [u8; MAX_REPLY_FRAME],
}

impl SessionHandler {
    /// Creates a handler for node `id`.
    #[must_use]
    pub const fn new(id: u8) -> Self {
        Self {
            id,
            decoder: Decoder::new(),
            in_session: false,
            rx: [0; MAX_COMMAND_FRAME],
            tx: [0; MAX_REPLY_FRAME],
        }
    }

    /// Whether a session is open. Page commands are rejected while `false`.
    #[must_use]
    pub const fn in_session(&self) -> bool {
        self.in_session
    }

    /// Feeds one received wire byte from the UART link.
    ///
    /// Returns a command when a complete, valid session command addressed to
    /// this node was decoded. Malformed or foreign frames produce nothing;
    /// the master's reply timeout drives recovery.
    pub fn decode(&mut self, byte: u8) -> Option<Command> {
        let Ok(Some(frame_len)) = self.decoder.feed(byte, &mut self.rx) else {
            return None;
        };
        let frame = self.rx.get(..frame_len)?;
        let packet = Packet::parse(frame).ok()?;
        if packet.id != self.id {
            return None;
        }
        match packet.kind {
            Kind::ProgSessionBegin => Some(Command::Begin(decode_begin(packet.payload)?)),
            Kind::ProgPageWrite => {
                let (addr, data) = decode_write(packet.payload)?;
                let len = data.len();
                self.tx.get_mut(..len)?.copy_from_slice(data);
                Some(Command::PageWrite { addr, len })
            }
            Kind::ProgPageRead => {
                let (addr, len) = decode_read(packet.payload)?;
                Some(Command::PageRead { addr, len })
            }
            Kind::ProgSessionEnd => Some(Command::End),
            _ => None,
        }
    }

    /// Runs `cmd` against `prog` and returns the raw reply frame, ready to
    /// COBS-encode onto the link.
    ///
    /// `prog` must be a programmer for the mux-selected target and the link
    /// must be in UPDI mode; only the decode path runs in UART mode.
    #[must_use]
    pub fn execute<L: UpdiLink>(&mut self, cmd: Command, prog: &mut TinyProgrammer<L>) -> &[u8] {
        match cmd {
            Command::Begin(target) => self.begin(target, prog),
            Command::PageWrite { addr, len } => self.page_write(addr, len, prog),
            Command::PageRead { addr, len } => self.page_read(addr, len, prog),
            Command::End => self.end(prog),
        }
    }

    /// Abandons an open session after a link idle timeout: resets the target
    /// out of programming mode and closes the session. A no-op when no
    /// session is open.
    pub fn expire<L: UpdiLink>(&mut self, prog: &mut TinyProgrammer<L>) {
        if self.in_session {
            let _ = prog.leave();
            self.in_session = false;
        }
    }

    fn begin<L: UpdiLink>(&mut self, target: SessionTarget, prog: &mut TinyProgrammer<L>) -> &[u8] {
        if target != SessionTarget::Cellagent {
            return self.status_reply(SessionStatus::NotSupported);
        }
        let result = prog.chip_erase().and_then(|()| prog.enter());
        match result {
            Ok(()) => {
                self.in_session = true;
                self.status_reply(SessionStatus::Ok)
            }
            Err(err) => self.status_reply(status_of(&err)),
        }
    }
    fn page_write<L: UpdiLink>(
        &mut self,
        addr: u16,
        len: usize,
        prog: &mut TinyProgrammer<L>,
    ) -> &[u8] {
        if !self.in_session {
            return self.page_status_reply(SessionStatus::BadState, addr);
        }
        let Some(data) = self.tx.get(..len) else {
            return self.page_status_reply(SessionStatus::InvalidAddr, addr);
        };
        let result = prog.write_flash(u32::from(addr), data);
        let status = result.map_or_else(|err| status_of(&err), |()| SessionStatus::Ok);
        self.page_status_reply(status, addr)
    }

    fn page_read<L: UpdiLink>(
        &mut self,
        addr: u16,
        len: u8,
        prog: &mut TinyProgrammer<L>,
    ) -> &[u8] {
        let len = usize::from(len);
        if !self.in_session {
            return self.page_status_reply(SessionStatus::BadState, addr);
        }
        if len == 0 || len > cellguard_protocol::PAGE_MAX {
            return self.page_status_reply(SessionStatus::InvalidAddr, addr);
        }
        // The command frame in `rx` is consumed; stage [status, addr, data]
        // there and build the reply packet from it into `tx`.
        let payload_len = 3 + len;
        if payload_len > MAX_COMMAND_FRAME {
            // Static buffer sizes make this unreachable.
            return self.page_status_reply(SessionStatus::InvalidAddr, addr);
        }
        let Self { id, rx, tx, .. } = self;
        let (payload, _) = rx.split_at_mut(payload_len);
        let (head, data) = payload.split_at_mut(3);
        let status = match prog.read_flash(u32::from(addr), data) {
            Ok(()) => SessionStatus::Ok,
            Err(err) => status_of(&err),
        };
        head.copy_from_slice(&encode_page_status(status, addr));
        write_reply(*id, Kind::ProgPageData, payload, tx)
    }

    fn end<L: UpdiLink>(&mut self, prog: &mut TinyProgrammer<L>) -> &[u8] {
        let status = if self.in_session {
            self.in_session = false;
            prog.leave()
                .map_or_else(|err| status_of(&err), |()| SessionStatus::Ok)
        } else {
            SessionStatus::Ok
        };
        self.status_reply(status)
    }

    /// Builds a `ProgSessionStatus` reply with a status byte only.
    fn status_reply(&mut self, status: SessionStatus) -> &[u8] {
        let payload = [status.to_code()];
        write_reply(self.id, Kind::ProgSessionStatus, &payload, &mut self.tx)
    }

    /// Builds a `ProgSessionStatus` reply to a page command.
    fn page_status_reply(&mut self, status: SessionStatus, addr: u16) -> &[u8] {
        let payload = encode_page_status(status, addr);
        write_reply(self.id, Kind::ProgSessionStatus, &payload, &mut self.tx)
    }
}

/// Builds a raw reply frame for `id` in `tx`. Sizes are static (see the
/// buffer consts), so this always succeeds; the fallback slice is empty.
fn write_reply<'t>(
    id: u8,
    kind: Kind,
    payload: &[u8],
    tx: &'t mut [u8; MAX_REPLY_FRAME],
) -> &'t [u8] {
    let Ok(len) = Packet::write(id, kind, payload, tx) else {
        return &[];
    };
    tx.get(..len).unwrap_or(&[])
}
#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use cellguard_protocol::{
        Kind, Packet, SessionStatus, SessionTarget, decode_page_data, decode_page_status,
        encode_begin, encode_frame, encode_read, encode_write,
    };
    use updi::TinyProgrammer;
    use updi::mock::MockTarget;

    use super::{Command, SessionHandler};

    const NODE: u8 = 5;

    struct Rig {
        handler: SessionHandler,
        target: TinyProgrammer<MockTarget>,
    }

    impl Rig {
        fn new(target: MockTarget) -> Self {
            Self {
                handler: SessionHandler::new(NODE),
                target: TinyProgrammer::new(target),
            }
        }

        fn target(self) -> MockTarget {
            self.target.free()
        }
    }

    /// Sends a raw frame to the handler, one byte at a time, collecting any
    /// decoded command.
    fn send(handler: &mut SessionHandler, raw: &[u8]) -> Option<Command> {
        let mut wire = [0u8; 96];
        let n = encode_frame(raw, &mut wire).expect("wire buffer fits");
        let mut cmd = None;
        for &byte in &wire[..n] {
            if let Some(c) = handler.decode(byte) {
                cmd = Some(c);
            }
        }
        cmd
    }

    fn begin_raw(target: SessionTarget) -> Vec<u8> {
        let mut raw = Vec::from([0u8; 16]);
        let n = Packet::write(
            NODE,
            Kind::ProgSessionBegin,
            &encode_begin(target),
            &mut raw,
        )
        .expect("fits");
        raw.truncate(n);
        raw
    }

    fn write_raw(addr: u16, data: &[u8]) -> Vec<u8> {
        let mut payload = [0u8; 2 + cellguard_protocol::PAGE_MAX];
        let pl = encode_write(addr, data, &mut payload).expect("payload fits");
        let mut raw = Vec::from([0u8; 96]);
        let n = Packet::write(NODE, Kind::ProgPageWrite, pl, &mut raw).expect("fits");
        raw.truncate(n);
        raw
    }

    fn read_raw(addr: u16, len: u8) -> Vec<u8> {
        let mut raw = Vec::from([0u8; 96]);
        let n = Packet::write(NODE, Kind::ProgPageRead, &encode_read(addr, len), &mut raw)
            .expect("fits");
        raw.truncate(n);
        raw
    }

    fn end_raw() -> Vec<u8> {
        let mut raw = Vec::from([0u8; 16]);
        let n = Packet::write(NODE, Kind::ProgSessionEnd, &[], &mut raw).expect("fits");
        raw.truncate(n);
        raw
    }

    /// Decodes a reply frame into (kind, payload).
    fn parse_reply(raw: &[u8]) -> (Kind, &[u8]) {
        let packet = Packet::parse(raw).expect("reply parses");
        (packet.kind, packet.payload)
    }

    #[test]
    fn happy_path_write_then_read_back_then_end() {
        let mut rig = Rig::new(MockTarget::tiny());
        // 68 bytes: spans 4 native pages (16 B) plus a partial fifth.
        let image: [u8; 68] = core::array::from_fn(|i| u8::try_from(41 + i).unwrap());

        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        assert_eq!(cmd, Command::Begin(SessionTarget::Cellagent));
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (kind, payload) = parse_reply(reply);
        assert_eq!(kind, Kind::ProgSessionStatus);
        assert_eq!(payload, &[SessionStatus::Ok.to_code()]);
        assert!(rig.handler.in_session());

        for (i, chunk) in image.chunks(32).enumerate() {
            let addr = u16::try_from(i * 32).expect("fits u16");
            let cmd = send(&mut rig.handler, &write_raw(addr, chunk)).expect("page write decodes");
            assert_eq!(
                cmd,
                Command::PageWrite {
                    addr,
                    len: chunk.len()
                }
            );
            let reply = rig.handler.execute(cmd, &mut rig.target);
            let (kind, payload) = parse_reply(reply);
            assert_eq!(kind, Kind::ProgSessionStatus);
            let (status, echo) = decode_page_status(payload).expect("status payload");
            assert_eq!(status, SessionStatus::Ok, "page {i} must write");
            assert_eq!(echo, addr);
        }

        for (i, chunk) in image.chunks(32).enumerate() {
            let addr = u16::try_from(i * 32).expect("fits u16");
            let len = chunk.len().try_into().expect("fits u8");
            let cmd = send(&mut rig.handler, &read_raw(addr, len)).expect("page read decodes");
            let reply = rig.handler.execute(cmd, &mut rig.target);
            let (kind, payload) = parse_reply(reply);
            assert_eq!(kind, Kind::ProgPageData);
            let (status, echo, data) = decode_page_data(payload).expect("data payload");
            assert_eq!(status, SessionStatus::Ok);
            assert_eq!(echo, addr);
            assert_eq!(data, chunk, "page {i} must read back");
        }

        let cmd = send(&mut rig.handler, &end_raw()).expect("end decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (kind, payload) = parse_reply(reply);
        assert_eq!(kind, Kind::ProgSessionStatus);
        assert_eq!(payload, &[SessionStatus::Ok.to_code()]);
        assert!(!rig.handler.in_session());
    }

    #[test]
    fn page_write_before_begin_is_rejected_and_flash_untouched() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd = send(&mut rig.handler, &write_raw(0, &[1, 2, 3, 4])).expect("page write decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (_, payload) = parse_reply(reply);
        let (status, _) = decode_page_status(payload).expect("status payload");
        assert_eq!(status, SessionStatus::BadState);
        let target = rig.target();
        for off in 0..64 {
            assert_eq!(
                target.flash_at(off),
                0xFF,
                "flash must stay erased at {off}"
            );
        }
    }

    #[test]
    fn page_read_before_begin_is_rejected() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd = send(&mut rig.handler, &read_raw(0, 4)).expect("page read decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (kind, payload) = parse_reply(reply);
        assert_eq!(kind, Kind::ProgSessionStatus, "error replies are short");
        let (status, _) = decode_page_status(payload).expect("status payload");
        assert_eq!(status, SessionStatus::BadState);
    }

    #[test]
    fn end_without_begin_is_a_harmless_ok() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd = send(&mut rig.handler, &end_raw()).expect("end decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (_, payload) = parse_reply(reply);
        assert_eq!(payload, &[SessionStatus::Ok.to_code()]);
    }

    #[test]
    fn begin_twice_restarts_from_blank_flash() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target);
        let cmd = send(&mut rig.handler, &write_raw(0, &[0xAA; 32])).expect("page write decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target);

        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (_, payload) = parse_reply(reply);
        assert_eq!(payload, &[SessionStatus::Ok.to_code()]);

        let target = rig.target();
        for off in 0..64 {
            assert_eq!(
                target.flash_at(off),
                0xFF,
                "re-begin must re-erase at {off}"
            );
        }
    }

    #[test]
    fn begin_on_locked_target_chip_erases_and_enters() {
        let mut rig = Rig::new(MockTarget::tiny_locked());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (_, payload) = parse_reply(reply);
        assert_eq!(payload, &[SessionStatus::Ok.to_code()]);
        assert!(rig.handler.in_session());
    }

    #[test]
    fn unsupported_target_is_rejected_without_state_change() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellcore)).expect("begin decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (_, payload) = parse_reply(reply);
        assert_eq!(payload, &[SessionStatus::NotSupported.to_code()]);
        assert!(!rig.handler.in_session());
    }

    #[test]
    fn odd_address_and_overflow_are_invalid() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target);

        let cmd = send(&mut rig.handler, &write_raw(3, &[1, 2])).expect("page write decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (status, _) = decode_page_status(parse_reply(reply).1).expect("status payload");
        assert_eq!(status, SessionStatus::InvalidAddr);

        // 4094 + 4 overflows the 4 KiB flash.
        let cmd = send(&mut rig.handler, &write_raw(4094, &[1, 2, 3, 4])).expect("decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (status, _) = decode_page_status(parse_reply(reply).1).expect("status payload");
        assert_eq!(status, SessionStatus::InvalidAddr);
    }

    #[test]
    fn zero_and_oversized_read_lengths_are_invalid() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target);

        let oversized = u8::try_from(cellguard_protocol::PAGE_MAX + 1).expect("fits u8");
        for bad in [0u8, oversized] {
            let cmd = send(&mut rig.handler, &read_raw(0, bad)).expect("decodes");
            let reply = rig.handler.execute(cmd, &mut rig.target);
            let (status, _) = decode_page_status(parse_reply(reply).1).expect("status");
            assert_eq!(status, SessionStatus::InvalidAddr);
        }
    }

    #[test]
    fn nvm_failure_maps_to_status() {
        let mut rig = Rig::new(MockTarget::tiny_failing());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target);
        let cmd = send(&mut rig.handler, &write_raw(0, &[1, 2])).expect("page write decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (status, _) = decode_page_status(parse_reply(reply).1).expect("status payload");
        assert_eq!(status, SessionStatus::NvmError);
    }

    #[test]
    fn foreign_node_and_malformed_frames_are_ignored() {
        let mut rig = Rig::new(MockTarget::tiny());
        // Addressed to another node.
        let mut foreign = Vec::from([0u8; 16]);
        let n = Packet::write(
            9,
            Kind::ProgSessionBegin,
            &encode_begin(SessionTarget::Cellagent),
            &mut foreign,
        )
        .expect("fits");
        foreign.truncate(n);
        assert!(send(&mut rig.handler, &foreign).is_none());
        // PageWrite with empty data cannot decode.
        let mut malformed = Vec::from([0u8; 96]);
        let n =
            Packet::write(NODE, Kind::ProgPageWrite, &[0x00, 0x10], &mut malformed).expect("fits");
        malformed.truncate(n);
        assert!(send(&mut rig.handler, &malformed).is_none());
        assert!(!rig.handler.in_session());
    }

    #[test]
    fn expire_leaves_and_closes_the_session() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target);
        assert!(rig.handler.in_session());
        rig.handler.expire(&mut rig.target);
        assert!(!rig.handler.in_session());
        // Outside a session, expire is a no-op.
        rig.handler.expire(&mut rig.target);

        // After expiry page commands are rejected again.
        let cmd = send(&mut rig.handler, &write_raw(0, &[1, 2])).expect("page write decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let (status, _) = decode_page_status(parse_reply(reply).1).expect("status payload");
        assert_eq!(status, SessionStatus::BadState);
    }

    #[test]
    fn buffers_are_sized_for_the_worst_case() {
        assert_eq!(
            super::MAX_COMMAND_FRAME,
            cellguard_protocol::HEADER_LEN
                + 2
                + cellguard_protocol::PAGE_MAX
                + cellguard_protocol::PAYLOAD_CRC_LEN
        );
        assert_eq!(
            super::MAX_REPLY_FRAME,
            cellguard_protocol::HEADER_LEN
                + 3
                + cellguard_protocol::PAGE_MAX
                + cellguard_protocol::PAYLOAD_CRC_LEN
        );
    }
}
