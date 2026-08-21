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
    Command as WireCommand, Decoder, Reply, SessionStatus, SessionTarget, decode_command,
    encode_reply,
};
use updi::{ProgError, TinyProgrammer, UpdiLink};

/// Decoded size of the largest command frame (`PageWrite`).
pub const MAX_COMMAND_FRAME: usize = 1 + 2 + cellguard_protocol::PAGE_MAX + CRC_LEN;

/// Decoded size of the largest reply frame (`PageData`).
pub const MAX_REPLY_FRAME: usize = 1 + 3 + cellguard_protocol::PAGE_MAX + CRC_LEN;

/// Length of the frame CRC in bytes.
const CRC_LEN: usize = 2;

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
        /// Data length. The bytes live in the handler.
        len: usize,
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
    decoder: Decoder,
    in_session: bool,
    /// Decoded command frames, reused as the page-read staging buffer.
    rx: [u8; MAX_COMMAND_FRAME],
    /// Page-write data and the raw reply frame, never both at once.
    tx: [u8; MAX_REPLY_FRAME],
}

impl Default for SessionHandler {
    fn default() -> Self {
        Self::new()
    }
}

impl SessionHandler {
    /// Creates a handler. The all-zero initializer lets a `static` handler
    /// land in `.bss` instead of carrying a flash image in `.data`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
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
    /// Returns a command when a complete, valid session command was decoded.
    /// Malformed or corrupt frames produce nothing; the master's reply
    /// timeout drives recovery.
    ///
    /// Out-of-line: the firmware feeds this from its event loop, and keeping
    /// the decode path (COBS decoder plus frame codec) out of the loop body
    /// relieves register pressure on small targets.
    #[inline(never)]
    pub fn decode(&mut self, byte: u8) -> Option<Command> {
        let Ok(Some(frame_len)) = self.decoder.feed(byte, &mut self.rx) else {
            return None;
        };
        let frame = self.rx.get(..frame_len)?;
        match decode_command(frame)? {
            WireCommand::Begin(target) => Some(Command::Begin(target)),
            WireCommand::PageWrite { addr, data } => {
                let len = data.len();
                // Byte loop, not `copy_from_slice`: the variable length would
                // link the generic `memcpy` helper on small targets.
                let dst = self.tx.get_mut(..len)?;
                for (dst, src) in dst.iter_mut().zip(data) {
                    *dst = *src;
                }
                Some(Command::PageWrite { addr, len })
            }
            #[cfg(feature = "page-read")]
            WireCommand::PageRead { addr, len } => Some(Command::PageRead { addr, len }),
            WireCommand::End => Some(Command::End),
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
            #[cfg(feature = "page-read")]
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
            return self.status_reply(SessionStatus::NotSupported, None);
        }
        let result = prog.chip_erase().and_then(|()| prog.enter());
        match result {
            Ok(()) => {
                self.in_session = true;
                self.status_reply(SessionStatus::Ok, None)
            }
            Err(err) => self.status_reply(status_of(&err), None),
        }
    }
    fn page_write<L: UpdiLink>(
        &mut self,
        addr: u16,
        len: usize,
        prog: &mut TinyProgrammer<L>,
    ) -> &[u8] {
        if !self.in_session {
            return self.status_reply(SessionStatus::BadState, Some(addr));
        }
        let Some(data) = self.tx.get(..len) else {
            return self.status_reply(SessionStatus::InvalidAddr, Some(addr));
        };
        let result = prog.write_flash(addr, data);
        let status = result.map_or_else(|err| status_of(&err), |()| SessionStatus::Ok);
        self.status_reply(status, Some(addr))
    }

    #[cfg(feature = "page-read")]
    fn page_read<L: UpdiLink>(
        &mut self,
        addr: u16,
        len: u8,
        prog: &mut TinyProgrammer<L>,
    ) -> &[u8] {
        let len = usize::from(len);
        if !self.in_session {
            return self.status_reply(SessionStatus::BadState, Some(addr));
        }
        if len == 0 || len > cellguard_protocol::PAGE_MAX {
            return self.status_reply(SessionStatus::InvalidAddr, Some(addr));
        }
        // The command frame in `rx` is consumed. Stage the data there and
        // build the reply frame from it into `tx`.
        let Self { rx, tx, .. } = self;
        let data = rx.get_mut(3..3 + len).unwrap_or(&mut []);
        let status = match prog.read_flash(addr, data) {
            Ok(()) => SessionStatus::Ok,
            Err(err) => status_of(&err),
        };
        write_reply(Reply::PageData { status, addr, data }, tx)
    }

    fn end<L: UpdiLink>(&mut self, prog: &mut TinyProgrammer<L>) -> &[u8] {
        let status = if self.in_session {
            self.in_session = false;
            prog.leave()
                .map_or_else(|err| status_of(&err), |()| SessionStatus::Ok)
        } else {
            SessionStatus::Ok
        };
        self.status_reply(status, None)
    }

    /// Builds a status reply. `addr` extends the payload with the address the
    /// status refers to, so the master can match the reply to its page
    /// command.
    fn status_reply(&mut self, status: SessionStatus, addr: Option<u16>) -> &[u8] {
        write_reply(Reply::Status { status, addr }, &mut self.tx)
    }
}

/// Builds a raw reply frame in `tx`. Buffer sizes are static (see the buffer
/// consts), so this always succeeds and the fallback slice is empty.
fn write_reply<'t>(reply: Reply<'_>, tx: &'t mut [u8; MAX_REPLY_FRAME]) -> &'t [u8] {
    let Some(len) = encode_reply(reply, tx) else {
        return &[];
    };
    tx.get(..len).unwrap_or(&[])
}
#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use cellguard_protocol::{
        Command as WireCommand, Reply, SessionStatus, SessionTarget, decode_reply, encode_command,
        encode_frame,
    };
    use updi::TinyProgrammer;
    use updi::mock::MockTarget;

    use super::{Command, SessionHandler};

    struct Rig {
        handler: SessionHandler,
        target: TinyProgrammer<MockTarget>,
    }

    impl Rig {
        fn new(target: MockTarget) -> Self {
            Self {
                handler: SessionHandler::new(),
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
        let n = encode_command(WireCommand::Begin(target), &mut raw).expect("fits");
        raw.truncate(n);
        raw
    }

    fn write_raw(addr: u16, data: &[u8]) -> Vec<u8> {
        let mut raw = Vec::from([0u8; 96]);
        let n = encode_command(WireCommand::PageWrite { addr, data }, &mut raw).expect("fits");
        raw.truncate(n);
        raw
    }

    #[cfg(feature = "page-read")]
    fn read_raw(addr: u16, len: u8) -> Vec<u8> {
        let mut raw = Vec::from([0u8; 16]);
        let n = encode_command(WireCommand::PageRead { addr, len }, &mut raw).expect("fits");
        raw.truncate(n);
        raw
    }

    fn end_raw() -> Vec<u8> {
        let mut raw = Vec::from([0u8; 16]);
        let n = encode_command(WireCommand::End, &mut raw).expect("fits");
        raw.truncate(n);
        raw
    }

    /// Decodes a reply frame into the reply.
    fn parse_reply(raw: &[u8]) -> Reply<'_> {
        decode_reply(raw).expect("reply parses")
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
        assert_eq!(
            parse_reply(reply),
            Reply::Status {
                status: SessionStatus::Ok,
                addr: None,
            }
        );
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
            assert_eq!(
                parse_reply(reply),
                Reply::Status {
                    status: SessionStatus::Ok,
                    addr: Some(addr),
                },
                "page {i} must write"
            );
        }

        #[cfg(feature = "page-read")]
        for (i, chunk) in image.chunks(32).enumerate() {
            let addr = u16::try_from(i * 32).expect("fits u16");
            let len = chunk.len().try_into().expect("fits u8");
            let cmd = send(&mut rig.handler, &read_raw(addr, len)).expect("page read decodes");
            let reply = rig.handler.execute(cmd, &mut rig.target);
            assert_eq!(
                parse_reply(reply),
                Reply::PageData {
                    status: SessionStatus::Ok,
                    addr,
                    data: chunk,
                },
                "page {i} must read back"
            );
        }

        let cmd = send(&mut rig.handler, &end_raw()).expect("end decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        assert_eq!(
            parse_reply(reply),
            Reply::Status {
                status: SessionStatus::Ok,
                addr: None,
            }
        );
        assert!(!rig.handler.in_session());
    }

    #[test]
    fn page_write_before_begin_is_rejected_and_flash_untouched() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd = send(&mut rig.handler, &write_raw(0, &[1, 2, 3, 4])).expect("page write decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
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
    #[cfg(feature = "page-read")]
    fn page_read_before_begin_is_rejected() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd = send(&mut rig.handler, &read_raw(0, 4)).expect("page read decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let Reply::Status { status, addr } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        assert_eq!(status, SessionStatus::BadState, "error replies are short");
        assert_eq!(addr, Some(0));
    }

    #[test]
    fn end_without_begin_is_a_harmless_ok() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd = send(&mut rig.handler, &end_raw()).expect("end decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        assert!(matches!(
            parse_reply(reply),
            Reply::Status {
                status: SessionStatus::Ok,
                addr: None
            }
        ));
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
        assert!(matches!(
            parse_reply(reply),
            Reply::Status {
                status: SessionStatus::Ok,
                addr: None
            }
        ));

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
        assert!(matches!(
            parse_reply(reply),
            Reply::Status {
                status: SessionStatus::Ok,
                addr: None
            }
        ));
        assert!(rig.handler.in_session());
    }

    #[test]
    fn unsupported_target_is_rejected_without_state_change() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellcore)).expect("begin decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        assert!(matches!(
            parse_reply(reply),
            Reply::Status {
                status: SessionStatus::NotSupported,
                addr: None
            }
        ));
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
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        assert_eq!(status, SessionStatus::InvalidAddr);

        // 4094 + 4 overflows the 4 KiB flash.
        let cmd = send(&mut rig.handler, &write_raw(4094, &[1, 2, 3, 4])).expect("decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target);
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        assert_eq!(status, SessionStatus::InvalidAddr);
    }

    #[test]
    #[cfg(feature = "page-read")]
    fn zero_and_oversized_read_lengths_are_invalid() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target);

        let oversized = u8::try_from(cellguard_protocol::PAGE_MAX + 1).expect("fits u8");
        for bad in [0u8, oversized] {
            let cmd = send(&mut rig.handler, &read_raw(0, bad)).expect("decodes");
            let reply = rig.handler.execute(cmd, &mut rig.target);
            let Reply::Status { status, .. } = parse_reply(reply) else {
                panic!("expected status reply")
            };
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
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        assert_eq!(status, SessionStatus::NvmError);
    }

    #[test]
    fn corrupt_and_malformed_frames_are_ignored() {
        let mut rig = Rig::new(MockTarget::tiny());
        // A flipped body byte fails the frame CRC.
        let mut corrupt = begin_raw(SessionTarget::Cellagent);
        corrupt[0] ^= 0x01;
        assert!(send(&mut rig.handler, &corrupt).is_none());
        // PageWrite with empty data cannot decode.
        let mut malformed = Vec::from([
            cellguard_protocol::SessionCmd::PageWrite.to_code(),
            0x00,
            0x10,
        ]);
        let body_crc = crc::checksum16(&malformed);
        malformed.extend_from_slice(&body_crc.to_le_bytes());
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
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        assert_eq!(status, SessionStatus::BadState);
    }

    #[test]
    fn buffers_are_sized_for_the_worst_case() {
        assert_eq!(
            super::MAX_COMMAND_FRAME,
            1 + 2 + cellguard_protocol::PAGE_MAX + 2
        );
        assert_eq!(
            super::MAX_REPLY_FRAME,
            1 + 3 + cellguard_protocol::PAGE_MAX + 2
        );
    }
}
