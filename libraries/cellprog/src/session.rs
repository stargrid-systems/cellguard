//! The servant-side programming session handler.
//!
//! The programmer's one USART is muxed between UART mode (decode and reply)
//! and UPDI mode (execute), so [`SessionHandler::decode`] and
//! [`SessionHandler::execute`] are separate calls. A decoded [`Command`]
//! must be executed before the next is decoded.
//!
//! # Self-update sessions
//!
//! A [`SessionTarget::CellprogSelf`] session updates the programmer itself.
//! It never touches the UPDI link: `Begin` arms without erasing anything,
//! page commands range-check only (the cellcore streams bytes it read from
//! the staging band, so they are already stored), and `End` CRC-checks the
//! whole staged image through [`SelfStaging`] before reporting `Ok`. On `Ok`
//! the handler latches [`SessionHandler::self_update_armed`]: the firmware
//! then sets the on-chip EEPROM update flag and resets, and the walker
//! applies the image on the next boot. A staged image that fails its check
//! never arms, so a corrupt image can never trigger the walker.

use cellboot::image::{MAGIC, Region};
use cellguard_protocol::{Command as WireCommand, Decoder, Reply, SessionStatus, SessionTarget};
use crc::Crc32;
use updi::{ProgError, TinyProgrammer, UpdiLink};

/// Decoded size of the largest command frame (`PageWrite`).
pub const MAX_COMMAND_FRAME: usize = 1 + 2 + cellguard_protocol::PAGE_MAX + CRC_LEN;

/// Decoded size of the largest reply frame (`PageData`).
pub const MAX_REPLY_FRAME: usize = 1 + 3 + cellguard_protocol::PAGE_MAX + CRC_LEN;

const CRC_LEN: usize = 2;

/// Size of the position-fixed walker region at the top of the cellprog
/// flash.
///
/// The walker is never rewritten by an update, so every update image ends
/// below it. See the firmware's `walker` module for the frozen ABI.
pub const WALKER_SIZE: u16 = 256;

/// Flash budget of the updatable application region: the 4 KiB flash minus
/// the walker region. Page commands beyond it are invalid, and the walker
/// walks exactly this many bytes.
pub const APP_FLASH_SIZE: u16 = updi::FLASH_SIZE - WALKER_SIZE;

/// Payload size the `End` verifier reads per store call. Matches the command
/// frame budget so the handler's receive buffer doubles as the scratch.
#[expect(
    clippy::cast_possible_truncation,
    reason = "the frame budget is far below u16 range"
)]
const VERIFY_CHUNK: u16 = cellguard_protocol::PAGE_MAX as u16;
/// Offset of the payload inside a staged image: right past the header.
#[expect(
    clippy::cast_possible_truncation,
    reason = "the header is far shorter than u16 range"
)]
const HEADER_OFFSET: u16 = cellboot::image::HEADER_LEN as u16;

const _: () = assert!(
    MAX_COMMAND_FRAME >= 3 + cellguard_protocol::PAGE_MAX,
    "rx must double as the page-read staging buffer"
);
const _: () = assert!(MAX_COMMAND_FRAME >= cellboot::image::HEADER_LEN);
const _: () = assert!(updi::FLASH_SIZE == 4096);

/// Read access to the staged cellprog self-update image.
///
/// The cellcore streams the image it already staged in the shared staging
/// EEPROM, so the bytes a `CellprogSelf` session carries are already in the
/// band by the time they arrive. The servant therefore does not write them
/// back: `End` re-reads the band through this trait and CRC-checks it before
/// arming the self-update, which verifies the servant's own read path end to
/// end. Implementations add the band's base offset.
pub trait SelfStaging {
    /// Fills `buf` with staged-image bytes, starting `offset` bytes into the
    /// image (offset 0 is the header). The offset is image-internal, so it
    /// always fits a `u16` (32-bit offset arithmetic is expensive on the
    /// servant's 8-bit target). Returns `false` when the store fails.
    fn read_staged(&mut self, offset: u16, buf: &mut [u8]) -> bool;
}

/// A decoded session command. Page data lives in the handler, not here.
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

/// The servant-side session state machine. See the module docs for the
/// firmware calling pattern.
pub struct SessionHandler {
    decoder: Decoder,
    in_session: bool,
    /// Whether the open session targets the programmer itself. Commands of a
    /// self session run against the staging store, never the UPDI link.
    self_session: bool,
    /// Set by `End` of a self session whose staged image verified. The
    /// firmware sets the on-chip update flag and resets after sending the
    /// reply.
    self_armed: bool,
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
    /// land in `.bss` instead of `.data`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            decoder: Decoder::new(),
            in_session: false,
            self_session: false,
            self_armed: false,
            rx: [0; MAX_COMMAND_FRAME],
            tx: [0; MAX_REPLY_FRAME],
        }
    }

    /// Whether a session is open. Page commands are rejected while `false`.
    #[must_use]
    pub const fn in_session(&self) -> bool {
        self.in_session
    }

    /// Whether a verified self-update is waiting to be applied. The firmware
    /// acts on this after sending the `End` reply: set the on-chip EEPROM
    /// update flag, then reset.
    #[must_use]
    pub const fn self_update_armed(&self) -> bool {
        self.self_armed
    }

    /// Whether executing `cmd` needs the link in UPDI mode. The firmware
    /// consults this before switching the mux: self-session commands run
    /// with the UART link connected.
    #[must_use]
    pub const fn uses_updi(&self, cmd: &Command) -> bool {
        match cmd {
            Command::Begin(SessionTarget::CellprogSelf) => false,
            Command::Begin(_) => true,
            Command::PageWrite { .. } | Command::End => !self.self_session,
            #[cfg(feature = "page-read")]
            Command::PageRead { .. } => !self.self_session,
        }
    }

    /// Feeds one received wire byte from the UART link.
    ///
    /// Malformed or corrupt frames produce nothing, and the master's reply
    /// timeout drives recovery. Kept out of line to relieve register
    /// pressure on small targets.
    #[inline(never)]
    pub fn decode(&mut self, byte: u8) -> Option<Command> {
        let Ok(Some(frame_len)) = self.decoder.feed(byte, &mut self.rx) else {
            return None;
        };
        let frame = self.rx.get(..frame_len)?;
        match WireCommand::decode(frame)? {
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

    /// Runs `cmd` against `prog` and returns the raw reply frame.
    ///
    /// The link must be in UPDI mode with the mux set to the target. A self
    /// session instead runs against `stage`, the staging-band store, and the
    /// link stays in UART mode.
    #[must_use]
    pub fn execute<L: UpdiLink, S: SelfStaging>(
        &mut self,
        cmd: Command,
        prog: &mut TinyProgrammer<L>,
        stage: &mut S,
    ) -> &[u8] {
        match cmd {
            Command::Begin(SessionTarget::CellprogSelf) => self.begin_self(),
            Command::Begin(target) => self.begin(target, prog),
            Command::PageWrite { addr, len } => {
                if self.self_session {
                    self.page_write_self(addr, len)
                } else {
                    self.page_write(addr, len, prog)
                }
            }
            #[cfg(feature = "page-read")]
            Command::PageRead { addr, len } => self.page_read(addr, len, prog),
            Command::End => {
                if self.self_session {
                    self.end_self(stage)
                } else {
                    self.end(prog)
                }
            }
        }
    }

    /// Abandons an open session after a link idle timeout, resetting the
    /// target out of programming mode. A no-op when no session is open.
    pub fn expire<L: UpdiLink>(&mut self, prog: &mut TinyProgrammer<L>) {
        if self.in_session {
            if !self.self_session {
                let _ = prog.leave();
            }
            self.in_session = false;
            self.self_session = false;
        }
    }

    fn begin_self(&mut self) -> &[u8] {
        self.in_session = true;
        self.self_session = true;
        self.status_reply(SessionStatus::Ok, None)
    }

    fn page_write_self(&mut self, addr: u16, len: usize) -> &[u8] {
        if !self.in_session {
            return self.status_reply(SessionStatus::BadState, Some(addr));
        }
        // The cellcore streams bytes it read from the staging band, so they
        // are already stored. Only the range is checked here: `End` verifies
        // the stored image before anything is armed.
        let end = addr.saturating_add(len.try_into().unwrap_or(u16::MAX));
        if end > APP_FLASH_SIZE {
            return self.status_reply(SessionStatus::InvalidAddr, Some(addr));
        }
        self.status_reply(SessionStatus::Ok, Some(addr))
    }

    fn end_self<S: SelfStaging>(&mut self, stage: &mut S) -> &[u8] {
        let ok = verify_staged(stage, &mut self.rx);
        if ok {
            self.self_armed = true;
        }
        self.in_session = false;
        self.self_session = false;
        self.status_reply(
            if ok {
                SessionStatus::Ok
            } else {
                SessionStatus::NvmError
            },
            None,
        )
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
        // The command frame in `rx` is consumed, so the read data can be
        // staged there.
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

    fn status_reply(&mut self, status: SessionStatus, addr: Option<u16>) -> &[u8] {
        write_reply(Reply::Status { status, addr }, &mut self.tx)
    }
}

/// Builds a raw reply frame in `tx`. The buffer is sized for the worst case,
/// so the empty-slice fallback is unreachable.
fn write_reply<'t>(reply: Reply<'_>, tx: &'t mut [u8; MAX_REPLY_FRAME]) -> &'t [u8] {
    let Some(len) = reply.encode(tx) else {
        return &[];
    };
    tx.get(..len).unwrap_or(&[])
}

/// Verifies the staged self-update image in `scratch`-sized reads: parses
/// enough header to route it, then CRC-32 checks the whole payload against
/// the header's own checksum.
///
/// `scratch` is the handler's receive buffer; the command frame it held is
/// consumed by the time `End` runs. Offsets stay `u16`: they are internal
/// to the image, whose payload `End` caps at the app budget.
fn verify_staged(stage: &mut impl SelfStaging, scratch: &mut [u8]) -> bool {
    let Some(head) = scratch.first_chunk_mut::<{ cellboot::image::HEADER_LEN }>() else {
        return false;
    };
    if !stage.read_staged(0, head) {
        return false;
    }
    // Field offsets from `cellboot::image::ImageHeader::serialize`.
    if head[0..4] != MAGIC
        || head[4] != cellboot::image::FORMAT_VERSION
        || Region::from_code(head[6]) != Some(Region::CellprogApp)
    {
        return false;
    }
    // The length field spans bytes 12-15. The high bytes must be zero:
    // otherwise the u16 read below truncates the claimed length and the
    // CRC would cover only a prefix of the staged image.
    let payload_len = u16::from_le_bytes([head[12], head[13]]);
    if payload_len == 0 || payload_len > APP_FLASH_SIZE || head[14] != 0 || head[15] != 0 {
        return false;
    }
    let expected = u32::from_le_bytes([head[16], head[17], head[18], head[19]]);

    let mut crc = Crc32::new();
    let mut done = HEADER_OFFSET;
    let mut rest = payload_len;
    while rest != 0 {
        let take = rest.min(VERIFY_CHUNK);
        let Some(buf) = scratch.get_mut(..usize::from(take)) else {
            return false;
        };
        if !stage.read_staged(done, buf) {
            return false;
        }
        crc.update(buf);
        done += take;
        rest -= take;
    }
    crc.finalize() == expected
}
#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use cellboot::image::{FORMAT_VERSION, HEADER_LEN, ImageHeader, ImageKind, Region};
    use cellguard_protocol::{
        Command as WireCommand, Reply, SessionStatus, SessionTarget, encode_frame,
    };
    use updi::TinyProgrammer;
    use updi::mock::MockTarget;

    use super::{Command, SessionHandler};

    struct Rig {
        handler: SessionHandler,
        target: TinyProgrammer<MockTarget>,
        band: FakeBand,
    }

    impl Rig {
        fn new(target: MockTarget) -> Self {
            Self {
                handler: SessionHandler::new(),
                target: TinyProgrammer::new(target),
                band: FakeBand::blank(),
            }
        }

        fn target(self) -> MockTarget {
            self.target.free()
        }
    }

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
        let n = WireCommand::Begin(target).encode(&mut raw).expect("fits");
        raw.truncate(n);
        raw
    }

    fn write_raw(addr: u16, data: &[u8]) -> Vec<u8> {
        let mut raw = Vec::from([0u8; 96]);
        let n = WireCommand::PageWrite { addr, data }
            .encode(&mut raw)
            .expect("fits");
        raw.truncate(n);
        raw
    }

    #[cfg(feature = "page-read")]
    fn read_raw(addr: u16, len: u8) -> Vec<u8> {
        let mut raw = Vec::from([0u8; 16]);
        let n = WireCommand::PageRead { addr, len }
            .encode(&mut raw)
            .expect("fits");
        raw.truncate(n);
        raw
    }

    fn end_raw() -> Vec<u8> {
        let mut raw = Vec::from([0u8; 16]);
        let n = WireCommand::End.encode(&mut raw).expect("fits");
        raw.truncate(n);
        raw
    }

    fn parse_reply(raw: &[u8]) -> Reply<'_> {
        Reply::decode(raw).expect("reply parses")
    }

    #[test]
    fn happy_path_write_then_read_back_then_end() {
        let mut rig = Rig::new(MockTarget::tiny());
        // 68 bytes: spans 4 native pages (16 B) plus a partial fifth.
        let image: [u8; 68] = core::array::from_fn(|i| u8::try_from(41 + i).unwrap());

        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        assert_eq!(cmd, Command::Begin(SessionTarget::Cellagent));
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
            let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
            let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        let cmd = send(&mut rig.handler, &write_raw(0, &[0xAA; 32])).expect("page write decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);

        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);

        let cmd = send(&mut rig.handler, &write_raw(3, &[1, 2])).expect("page write decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        assert_eq!(status, SessionStatus::InvalidAddr);

        // 4094 + 4 overflows the 4 KiB flash.
        let cmd = send(&mut rig.handler, &write_raw(4094, &[1, 2, 3, 4])).expect("decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);

        let oversized = u8::try_from(cellguard_protocol::PAGE_MAX + 1).expect("fits u8");
        for bad in [0u8, oversized] {
            let cmd = send(&mut rig.handler, &read_raw(0, bad)).expect("decodes");
            let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        let cmd = send(&mut rig.handler, &write_raw(0, &[1, 2])).expect("page write decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        assert_eq!(status, SessionStatus::NvmError);
    }

    #[test]
    fn corrupt_and_malformed_frames_are_ignored() {
        let mut rig = Rig::new(MockTarget::tiny());
        let mut corrupt = begin_raw(SessionTarget::Cellagent);
        corrupt[0] ^= 0x01;
        assert!(send(&mut rig.handler, &corrupt).is_none());
        let mut malformed = Vec::from([
            2, // PageWrite command byte
            0x00, 0x10,
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
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        assert!(rig.handler.in_session());
        rig.handler.expire(&mut rig.target);
        assert!(!rig.handler.in_session());
        rig.handler.expire(&mut rig.target);

        let cmd = send(&mut rig.handler, &write_raw(0, &[1, 2])).expect("page write decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
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

    /// An in-memory staging band: the image the cellcore staged, with
    /// optional read failures after a set number of reads.
    struct FakeBand {
        image: Vec<u8>,
        fail_after: Option<usize>,
        reads: usize,
    }

    impl FakeBand {
        fn blank() -> Self {
            Self {
                image: Vec::new(),
                fail_after: None,
                reads: 0,
            }
        }

        /// Stages a valid cellprog image carrying `payload`.
        fn stage(&mut self, payload: &[u8]) {
            self.stage_as(Region::CellprogApp, payload);
        }

        /// Stages an image for `region`, however mismatched.
        fn stage_as(&mut self, region: Region, payload: &[u8]) {
            let header = ImageHeader {
                kind: ImageKind::Application,
                region,
                target_id: 3,
                fw_version: 4,
                payload_len: u32::try_from(payload.len()).unwrap(),
                payload_crc32: crc::checksum32(payload),
                hmac: [0u8; 32],
            };
            let mut image = header.serialize().to_vec();
            image.extend_from_slice(payload);
            self.image = image;
        }
    }

    impl super::SelfStaging for FakeBand {
        fn read_staged(&mut self, offset: u16, buf: &mut [u8]) -> bool {
            self.reads += 1;
            if self.fail_after.is_some_and(|limit| self.reads > limit) {
                return false;
            }
            let start = usize::from(offset);
            let Some(bytes) = self.image.get(start..start + buf.len()) else {
                return false;
            };
            buf.copy_from_slice(bytes);
            true
        }
    }

    /// Runs a full self session against the rig's band and returns the `End`
    /// status.
    fn run_self_session(rig: &mut Rig, payload: &[u8]) -> SessionStatus {
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::CellprogSelf)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        for (i, chunk) in payload.chunks(32).enumerate() {
            let addr = u16::try_from(i * 32).expect("fits u16");
            let cmd = send(&mut rig.handler, &write_raw(addr, chunk)).expect("decodes");
            let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        }
        let cmd = send(&mut rig.handler, &end_raw()).expect("end decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        status
    }

    #[test]
    fn self_session_verifies_and_arms() {
        let payload: Vec<u8> = (0..150u32).map(|i| u8::try_from(i).unwrap()).collect();
        let mut rig = Rig::new(MockTarget::tiny());
        rig.band.stage(&payload);

        assert_eq!(run_self_session(&mut rig, &payload), SessionStatus::Ok);
        assert!(rig.handler.self_update_armed());
        assert!(!rig.handler.in_session());
        // The servant never talks UPDI in a self session, so the target is
        // untouched.
        assert_eq!(rig.target().flash_at(0), 0xFF);
    }

    #[test]
    fn corrupt_staged_image_never_arms() {
        let payload = [0x77u8; 130];
        let mut rig = Rig::new(MockTarget::tiny());
        rig.band.stage(&payload);
        // Flip one payload byte after staging.
        if let Some(byte) = rig.band.image.get_mut(HEADER_LEN + 3) {
            *byte ^= 0x01;
        }

        assert_eq!(
            run_self_session(&mut rig, &payload),
            SessionStatus::NvmError
        );
        assert!(!rig.handler.self_update_armed());
    }

    #[test]
    fn foreign_region_never_arms() {
        let payload = [1u8, 2, 3];
        let mut rig = Rig::new(MockTarget::tiny());
        rig.band.stage_as(Region::CellagentApp, &payload);

        assert_eq!(
            run_self_session(&mut rig, &payload),
            SessionStatus::NvmError
        );
        assert!(!rig.handler.self_update_armed());
    }

    #[test]
    fn oversized_payload_never_arms() {
        let payload = [1u8, 2, 3];
        let mut rig = Rig::new(MockTarget::tiny());
        rig.band.stage(&payload);
        // Patch the header length beyond the app budget. The CRC no longer
        // needs to match: the length check fires first.
        let len = u32::from(super::APP_FLASH_SIZE) + 1;
        rig.band.image[12..16].copy_from_slice(&len.to_le_bytes());

        assert_eq!(
            run_self_session(&mut rig, &payload),
            SessionStatus::NvmError
        );
        assert!(!rig.handler.self_update_armed());
    }

    #[test]
    fn high_length_bytes_never_arms() {
        // Claim 0x1_0100 bytes. The low 16 bits stay in budget and the
        // CRC covers all 256 payload bytes, so only the high length
        // bytes reject this.
        let payload = [0x33u8; 0x0100];
        let mut rig = Rig::new(MockTarget::tiny());
        rig.band.stage(&payload);
        rig.band.image[12..16].copy_from_slice(&0x0001_0100u32.to_le_bytes());

        assert_eq!(
            run_self_session(&mut rig, &payload),
            SessionStatus::NvmError
        );
        assert!(!rig.handler.self_update_armed());
    }

    #[test]
    fn full_budget_length_still_arms() {
        // An honest header at the full app budget has zero high length
        // bytes and must still verify.
        let payload = std::vec![0x55u8; usize::from(super::APP_FLASH_SIZE)];
        let mut rig = Rig::new(MockTarget::tiny());
        rig.band.stage(&payload);

        assert_eq!(run_self_session(&mut rig, &payload), SessionStatus::Ok);
        assert!(rig.handler.self_update_armed());
    }

    #[test]
    fn empty_payload_never_arms() {
        // An empty image would arm a walk that erases the whole app region.
        let mut rig = Rig::new(MockTarget::tiny());
        rig.band.stage(&[]);

        assert_eq!(run_self_session(&mut rig, &[]), SessionStatus::NvmError);
        assert!(!rig.handler.self_update_armed());
    }

    #[test]
    fn unreadable_band_never_arms() {
        let payload = [5u8; 10];
        let mut rig = Rig::new(MockTarget::tiny());
        rig.band.stage(&payload);
        rig.band.fail_after = Some(0);

        assert_eq!(
            run_self_session(&mut rig, &payload),
            SessionStatus::NvmError
        );
        assert!(!rig.handler.self_update_armed());
    }

    #[test]
    fn self_page_out_of_budget_is_invalid() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::CellprogSelf)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);

        let at = super::APP_FLASH_SIZE;
        let cmd = send(&mut rig.handler, &write_raw(at - 1, &[1, 2])).expect("decodes");
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        assert_eq!(status, SessionStatus::InvalidAddr);
    }

    #[test]
    fn self_page_before_begin_is_rejected() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd = send(&mut rig.handler, &write_raw(0, &[1, 2])).expect("decodes");
        // The session is not open, so the command is not a self command yet.
        let reply = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        let Reply::Status { status, .. } = parse_reply(reply) else {
            panic!("expected status reply")
        };
        assert_eq!(status, SessionStatus::BadState);
    }

    #[test]
    fn uses_updi_routes_self_commands_to_the_uart_link() {
        let page = Command::PageWrite { addr: 0, len: 2 };
        let self_begin = Command::Begin(SessionTarget::CellprogSelf);
        let mut rig = Rig::new(MockTarget::tiny());
        assert!(!rig.handler.uses_updi(&self_begin));
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::CellprogSelf)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        assert!(!rig.handler.uses_updi(&page));
        assert!(!rig.handler.uses_updi(&Command::End));

        // Outside a self session everything needs the UPDI link.
        let agent_begin = Command::Begin(SessionTarget::Cellagent);
        let mut rig = Rig::new(MockTarget::tiny());
        assert!(rig.handler.uses_updi(&agent_begin));
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::Cellagent)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        assert!(rig.handler.uses_updi(&page));
        assert!(rig.handler.uses_updi(&Command::End));
    }

    #[test]
    fn expire_closes_a_self_session_without_arming() {
        let mut rig = Rig::new(MockTarget::tiny());
        let cmd =
            send(&mut rig.handler, &begin_raw(SessionTarget::CellprogSelf)).expect("begin decodes");
        let _ = rig.handler.execute(cmd, &mut rig.target, &mut rig.band);
        rig.handler.expire(&mut rig.target);
        assert!(!rig.handler.in_session());
        assert!(!rig.handler.self_update_armed());
    }

    #[test]
    fn self_session_flashes_nothing_over_updi() {
        // A whole self session must leave the (updi-side) target erased.
        let payload = [0x42u8; 64];
        let mut rig = Rig::new(MockTarget::tiny());
        rig.band.stage(&payload);
        let _ = run_self_session(&mut rig, &payload);
        let target = rig.target();
        for off in [0usize, 16, 32] {
            assert_eq!(target.flash_at(off), 0xFF, "flash stays erased at {off}");
        }
    }

    #[test]
    fn app_flash_budget_excludes_the_walker() {
        assert_eq!(super::APP_FLASH_SIZE, 4096 - 256);
        assert_eq!(super::APP_FLASH_SIZE % 16, 0, "whole flash pages");
    }

    #[test]
    fn header_field_offsets_match_cellboot() {
        // The hand-rolled field reads in `verify_staged` must track
        // `ImageHeader::serialize`.
        let header = ImageHeader {
            kind: ImageKind::Application,
            region: Region::CellprogApp,
            target_id: 1,
            fw_version: 9,
            payload_len: 0x0102_0304,
            payload_crc32: 0x0A0B_0C0D,
            hmac: [0u8; 32],
        };
        let raw = header.serialize();
        assert_eq!(&raw[0..4], b"CGFW");
        assert_eq!(raw[4], FORMAT_VERSION);
        assert_eq!(raw[6], Region::CellprogApp.to_code());
        assert_eq!(raw[12..16], 0x0102_0304u32.to_le_bytes());
        assert_eq!(raw[16..20], 0x0A0B_0C0Du32.to_le_bytes());
    }
}
