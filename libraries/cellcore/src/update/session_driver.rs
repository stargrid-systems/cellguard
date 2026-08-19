//! The master-side programming-session driver.
//!
//! [`SessionDriver`] is the cellcore's half of the transactional programming
//! protocol (see `cellguard_protocol::session`). Every
//! [`Command::PageWrite`] carries its bytes over the link, so the master
//! streams the image from its own staging store through [`StagedImage`].
//!
//! One session is [`Command::Begin`], one page command per
//! [`PAGE_MAX`]-byte chunk of the payload, then [`Command::End`].
//! [`SessionDriver::start`] verifies the staged copy (header parse plus
//! payload CRC-32) before anything is sent, because `Begin` chip-erases the
//! target: a corrupt staged image must never destroy a working one.
//!
//! The driver never blocks: [`SessionDriver::pump`] sends at most one
//! command, then drains reply bytes that have already arrived, and returns.
//! A lost reply is retried a bounded number of times: page commands are
//! idempotent, and a re-sent `Begin` restarts from blank flash. A non-`Ok`
//! status reply is authoritative and fails the session without retry.
//!
//! The cellagent and the programmer are flashed over this link
//! ([`target_for`]): the programmer receives a `CellprogSelf` session, which
//! stages the image and rewrites itself after reset. An application image
//! stays staged for the bootloader's self-program path, and a bootloader
//! image is bench-only.

use cellboot::image::{HEADER_LEN, HEADER_LEN_U32, ImageHeader, Region};
use cellguard_protocol::{
    Command, Decoder, MAX_COMMAND_WIRE, PAGE_MAX, Reply, SessionStatus, SessionTarget,
    decode_reply, encode_command, encode_frame,
};
use crc::Crc32;
use embedded_io::{Read, Write};

/// Decoded size of the largest command frame (`PageWrite`).
const MAX_COMMAND_FRAME: usize = 1 + 2 + PAGE_MAX + 2;

/// Decoded size of the largest reply frame (`PageData`).
const MAX_REPLY_FRAME: usize = 1 + 3 + PAGE_MAX + 2;

/// Sends per command: one attempt plus retries for a reply lost on the link.
pub const MAX_ATTEMPTS: u8 = 3;

/// Pumps that produced no complete reply before the attempt is retried (or
/// the session fails on the last attempt).
///
/// One pump waits out one link receive timeout, so at the firmware's 5 ms
/// timeout this bounds a command's wait at roughly a third of a second per
/// attempt.
pub const REPLY_WAIT_PUMPS: u16 = 64;

/// Reads the staged image a session streams.
///
/// Implemented by [`UpdateAgent`](crate::update::session::UpdateAgent), whose
/// store holds the committed image.
pub trait StagedImage {
    /// Fills `buf` with image bytes for `region`, starting at `offset` from
    /// the image header. Returns `false` when the region is not staged or
    /// the store fails.
    fn read_staged(&mut self, region: Region, offset: u32, buf: &mut [u8]) -> bool;
}

/// Maps a committed region to the session target that can flash it.
///
/// Returns `None` for every region the programmer link cannot program: the
/// application region belongs to the bootloader's self-program path, the
/// bootloader region is bench-only, and the factory region is not a firmware
/// target.
#[must_use]
pub const fn target_for(region: Region) -> Option<SessionTarget> {
    match region {
        Region::CellagentApp => Some(SessionTarget::Cellagent),
        Region::CellprogApp => Some(SessionTarget::CellprogSelf),
        _ => None,
    }
}

/// Why a session failed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SessionFailure {
    /// The staged image could not be read, parsed, or failed its CRC check.
    /// Detected before `Begin`, so the target is never erased for a bad
    /// image.
    CorruptSource,
    /// A command could not be written to the programmer link.
    LinkWrite,
    /// No usable reply arrived within the retry budget: the link stayed
    /// silent, or every reply was corrupt or mismatched.
    ReplyTimeout,
    /// The programmer rejected a command with this status. Authoritative:
    /// the operation ran and failed, so it is not retried.
    Rejected(SessionStatus),
}

/// The outcome of one [`SessionDriver::pump`] call.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Progress {
    /// No session is running.
    Idle,
    /// The session advanced but has not finished.
    Pending,
    /// The target was programmed and released.
    Success,
    /// The session failed. Reported once, then the driver goes idle.
    Failed(SessionFailure),
}

/// The command currently in flight.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Step {
    /// Chip-erase the target and enter programming mode.
    Begin,
    /// Program the loaded page at `addr`.
    Page { addr: u16 },
    /// Leave programming mode and reset the target.
    End,
}

enum Phase {
    Idle,
    InFlight,
}

/// The master-side programming-session driver. See the
/// [module](self) docs for the protocol and the driving model.
pub struct SessionDriver {
    phase: Phase,
    target: SessionTarget,
    region: Region,
    step: Step,
    /// Payload bytes already loaded into pages.
    loaded: u32,
    /// Payload bytes not yet loaded into a page.
    remaining: u32,
    /// Length of the page currently loaded.
    page_len: usize,
    /// Sends of the current step so far.
    attempts: u8,
    /// Pumps since the last send that produced no complete reply.
    quiet_pumps: u16,
    decoder: Decoder,
    /// Decoded reply frame.
    rx: [u8; MAX_REPLY_FRAME],
    /// The page being streamed.
    page: [u8; PAGE_MAX],
    /// One encoded command frame, before COBS.
    raw: [u8; MAX_COMMAND_FRAME],
    /// One encoded command frame, after COBS.
    wire: [u8; MAX_COMMAND_WIRE],
}

impl Default for SessionDriver {
    fn default() -> Self {
        Self::new()
    }
}

impl SessionDriver {
    /// Creates an idle driver. The all-zero initializer lets a `static`
    /// driver land in `.bss` instead of carrying a flash image in `.data`.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            phase: Phase::Idle,
            target: SessionTarget::Cellagent,
            region: Region::CellagentApp,
            step: Step::Begin,
            loaded: 0,
            remaining: 0,
            page_len: 0,
            attempts: 0,
            quiet_pumps: 0,
            decoder: Decoder::new(),
            rx: [0; MAX_REPLY_FRAME],
            page: [0; PAGE_MAX],
            raw: [0; MAX_COMMAND_FRAME],
            wire: [0; MAX_COMMAND_WIRE],
        }
    }

    /// Whether no session is running.
    #[must_use]
    pub const fn idle(&self) -> bool {
        matches!(self.phase, Phase::Idle)
    }

    /// Verifies the staged image for `region` and arms a session against
    /// `target`.
    ///
    /// Reads the header and CRCs the whole staged payload before anything is
    /// sent, because `Begin` chip-erases the target: a corrupt EEPROM must
    /// never destroy a working target. This pass blocks for the duration of
    /// one store read per page.
    ///
    /// Must only be called while idle. On error the driver stays idle.
    ///
    /// # Errors
    ///
    /// Returns [`SessionFailure::CorruptSource`] when the image cannot be
    /// read or fails its checks, and [`SessionFailure::Rejected`] when the
    /// payload cannot fit the protocol's `u16` addresses.
    pub fn start(
        &mut self,
        target: SessionTarget,
        region: Region,
        image: &mut impl StagedImage,
    ) -> Result<(), SessionFailure> {
        let mut header_bytes = [0u8; HEADER_LEN];
        if !image.read_staged(region, 0, &mut header_bytes) {
            return Err(SessionFailure::CorruptSource);
        }
        let header =
            ImageHeader::parse(&header_bytes).map_err(|_| SessionFailure::CorruptSource)?;
        if u16::try_from(header.payload_len).is_err() {
            return Err(SessionFailure::Rejected(SessionStatus::InvalidAddr));
        }
        let mut crc = Crc32::new();
        let mut done = 0u32;
        while done < header.payload_len {
            let take = u32::try_from(PAGE_MAX)
                .unwrap_or(u32::MAX)
                .min(header.payload_len - done);
            let n = usize::try_from(take).unwrap_or(PAGE_MAX);
            let Some(buf) = self.page.get_mut(..n) else {
                return Err(SessionFailure::CorruptSource);
            };
            let offset = HEADER_LEN_U32.saturating_add(done);
            if !image.read_staged(region, offset, buf) {
                return Err(SessionFailure::CorruptSource);
            }
            crc.update(buf);
            done = done.saturating_add(take);
        }
        if crc.finalize() != header.payload_crc32 {
            return Err(SessionFailure::CorruptSource);
        }

        self.phase = Phase::InFlight;
        self.target = target;
        self.region = region;
        self.step = Step::Begin;
        self.loaded = 0;
        self.remaining = header.payload_len;
        self.page_len = 0;
        self.attempts = 0;
        self.quiet_pumps = 0;
        self.decoder = Decoder::new();
        Ok(())
    }

    /// Advances the running session by one step. See the
    /// [module](self) docs for the driving model.
    ///
    /// Returns [`Progress::Idle`] when no session is running, and reports a
    /// terminal outcome exactly once per session.
    pub fn pump<L: Read + Write>(
        &mut self,
        link: &mut L,
        image: &mut impl StagedImage,
    ) -> Progress {
        if matches!(self.phase, Phase::Idle) {
            return Progress::Idle;
        }
        if self.attempts == 0 {
            return self.send(link);
        }
        // Drain reply bytes that have already arrived. The first read waits
        // out one link receive timeout, so an idle link returns here and the
        // caller's event loop keeps running.
        loop {
            let mut byte = [0u8; 1];
            if link.read_exact(&mut byte).is_err() {
                break;
            }
            if let Ok(Some(n)) = self.decoder.feed(byte[0], &mut self.rx)
                && let Some(reply) = self.rx.get(..n).and_then(decode_reply).and_then(status_of)
            {
                return self.on_status(reply, link, image);
            }
        }
        self.quiet_pumps = self.quiet_pumps.saturating_add(1);
        if self.quiet_pumps >= REPLY_WAIT_PUMPS {
            self.quiet_pumps = 0;
            return self.retry(link);
        }
        Progress::Pending
    }

    /// Handles the status of one complete reply, matched to the command in
    /// flight.
    fn on_status<L: Read + Write>(
        &mut self,
        reply: (SessionStatus, Option<u16>),
        link: &mut L,
        image: &mut impl StagedImage,
    ) -> Progress {
        let (status, addr) = reply;
        let expected = match self.step {
            Step::Page { addr } => Some(addr),
            Step::Begin | Step::End => None,
        };
        if addr != expected {
            return self.retry(link);
        }
        if status != SessionStatus::Ok {
            return self.fail(SessionFailure::Rejected(status));
        }
        self.advance(link, image)
    }

    /// Moves to the next command after an `Ok` reply.
    fn advance<L: Read + Write>(&mut self, link: &mut L, image: &mut impl StagedImage) -> Progress {
        let next = match self.step {
            Step::Begin => {
                if self.remaining == 0 {
                    Step::End
                } else {
                    Step::Page { addr: 0 }
                }
            }
            Step::Page { addr } => {
                if self.remaining == 0 {
                    Step::End
                } else {
                    let next_addr = addr.saturating_add(u16::try_from(self.page_len).unwrap_or(0));
                    Step::Page { addr: next_addr }
                }
            }
            Step::End => {
                self.phase = Phase::Idle;
                return Progress::Success;
            }
        };
        self.enter(next, link, image)
    }

    /// Enters `step`, loading its page if it has one, and sends it.
    fn enter<L: Read + Write>(
        &mut self,
        step: Step,
        link: &mut L,
        image: &mut impl StagedImage,
    ) -> Progress {
        self.step = step;
        self.attempts = 0;
        self.quiet_pumps = 0;
        if matches!(self.step, Step::Page { .. }) && !self.load_page(image) {
            return self.fail(SessionFailure::CorruptSource);
        }
        self.send(link)
    }

    /// Loads the next page from the store into the page buffer.
    fn load_page(&mut self, image: &mut impl StagedImage) -> bool {
        let take = u32::try_from(PAGE_MAX)
            .unwrap_or(u32::MAX)
            .min(self.remaining);
        let n = usize::try_from(take).unwrap_or(PAGE_MAX);
        let Some(buf) = self.page.get_mut(..n) else {
            return false;
        };
        let offset = HEADER_LEN_U32.saturating_add(self.loaded);
        if !image.read_staged(self.region, offset, buf) {
            return false;
        }
        self.page_len = n;
        self.loaded = self.loaded.saturating_add(take);
        self.remaining = self.remaining.saturating_sub(take);
        true
    }

    /// Encodes and writes the current step's command.
    fn send<L: Read + Write>(&mut self, link: &mut L) -> Progress {
        let cmd = match self.step {
            Step::Begin => Command::Begin(self.target),
            Step::Page { addr } => Command::PageWrite {
                addr,
                data: self.page.get(..self.page_len).unwrap_or(&[]),
            },
            Step::End => Command::End,
        };
        let raw_len = encode_command(cmd, &mut self.raw);
        let wire_len = raw_len
            .and_then(|n| self.raw.get(..n))
            .and_then(|raw| encode_frame(raw, &mut self.wire));
        let Some(bytes) = wire_len.and_then(|n| self.wire.get(..n)) else {
            return self.fail(SessionFailure::LinkWrite);
        };
        self.attempts = self.attempts.saturating_add(1);
        self.quiet_pumps = 0;
        self.decoder = Decoder::new();
        if link.write_all(bytes).is_err() || link.flush().is_err() {
            return self.fail(SessionFailure::LinkWrite);
        }
        Progress::Pending
    }

    /// Re-sends the current command, or fails the session when the attempt
    /// budget is spent.
    fn retry<L: Read + Write>(&mut self, link: &mut L) -> Progress {
        if self.attempts >= MAX_ATTEMPTS {
            return self.fail(SessionFailure::ReplyTimeout);
        }
        self.send(link)
    }

    /// Ends the session with `failure` and returns it.
    const fn fail(&mut self, failure: SessionFailure) -> Progress {
        self.phase = Phase::Idle;
        Progress::Failed(failure)
    }
}

/// Reduces a decoded reply to its status and the address it refers to. Any
/// other reply frame is foreign to this driver and counts as no reply.
const fn status_of(reply: Reply<'_>) -> Option<(SessionStatus, Option<u16>)> {
    if let Reply::Status { status, addr } = reply {
        Some((status, addr))
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use cellboot::image::{ImageHeader, ImageKind, Region};
    use cellguard_protocol::{
        Command as WireCommand, Decoder, PAGE_MAX, Reply, SessionCmd, SessionStatus, SessionTarget,
        decode_command, encode_frame, encode_reply,
    };

    use super::{
        MAX_ATTEMPTS, Progress, REPLY_WAIT_PUMPS, SessionDriver, SessionFailure, StagedImage,
        target_for,
    };

    /// A staging store holding one image, with optional read failures after
    /// a set number of reads.
    struct Source {
        image: Vec<u8>,
        fail_after: Option<usize>,
        reads: usize,
    }

    impl Source {
        fn new(payload: &[u8]) -> Self {
            let header = ImageHeader {
                kind: ImageKind::Application,
                region: Region::CellagentApp,
                target_id: 1,
                fw_version: 3,
                payload_len: u32::try_from(payload.len()).unwrap(),
                payload_crc32: crc::checksum32(payload),
                hmac: [0u8; 32],
            };
            let mut image = header.serialize().to_vec();
            image.extend_from_slice(payload);
            Self {
                image,
                fail_after: None,
                reads: 0,
            }
        }
    }

    impl StagedImage for Source {
        fn read_staged(&mut self, _region: Region, offset: u32, buf: &mut [u8]) -> bool {
            self.reads += 1;
            if self.fail_after.is_some_and(|limit| self.reads > limit) {
                return false;
            }
            let start = usize::try_from(offset).unwrap();
            let Some(bytes) = self.image.get(start..start + buf.len()) else {
                return false;
            };
            buf.copy_from_slice(bytes);
            true
        }
    }

    /// One servant-observed command.
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    struct Logged {
        cmd: SessionCmd,
        addr: Option<u16>,
        len: usize,
    }

    /// A decoded command, copied out of the decode buffer so the servant can
    /// mutate itself while executing it.
    enum Owned {
        Begin,
        PageWrite { addr: u16, data: Vec<u8> },
        End,
    }

    /// A programmer-servant link: decodes written commands with the real
    /// codec, runs a minimal servant against a fake flash, and queues real
    /// reply frames.
    struct Servant {
        written: Vec<u8>,
        readable: Vec<u8>,
        decoder: Decoder,
        scratch: [u8; 96],
        in_session: bool,
        flash: Vec<u8>,
        log: Vec<Logged>,
        /// Status to reply to page commands instead of writing flash.
        fail_status: Option<SessionStatus>,
        /// Replies to swallow, simulating transport loss.
        drop_replies: usize,
        /// Reply a wrong address for page commands.
        wrong_addr: bool,
    }

    impl Servant {
        fn new() -> Self {
            Self {
                written: Vec::new(),
                readable: Vec::new(),
                decoder: Decoder::new(),
                scratch: [0; 96],
                in_session: false,
                flash: Vec::new(),
                log: Vec::new(),
                fail_status: None,
                drop_replies: 0,
                wrong_addr: false,
            }
        }

        /// Feeds one written wire byte, servicing any completed command.
        fn on_byte(&mut self, byte: u8) {
            let Ok(Some(n)) = self.decoder.feed(byte, &mut self.scratch) else {
                return;
            };
            let Some(frame) = self.scratch.get(..n) else {
                return;
            };
            let owned = match decode_command(frame) {
                Some(WireCommand::Begin(_)) => Owned::Begin,
                Some(WireCommand::PageWrite { addr, data }) => Owned::PageWrite {
                    addr,
                    data: data.to_vec(),
                },
                Some(WireCommand::End) => Owned::End,
                _ => unreachable!("the driver never sends other commands"),
            };
            let (logged, reply) = self.execute(owned);
            self.log.push(logged);
            let mut raw = [0u8; 1 + 3 + PAGE_MAX + 2];
            let n = encode_reply(reply, &mut raw).unwrap();
            let mut wire = [0u8; 96];
            let wire_len = encode_frame(&raw[..n], &mut wire).unwrap();
            if self.drop_replies > 0 {
                self.drop_replies -= 1;
                return;
            }
            self.readable.extend_from_slice(&wire[..wire_len]);
        }

        /// Runs one decoded command against the fake flash.
        fn execute(&mut self, cmd: Owned) -> (Logged, Reply<'static>) {
            match cmd {
                Owned::Begin => {
                    self.in_session = true;
                    Self::reply_of(SessionCmd::Begin, None, 0, SessionStatus::Ok)
                }
                Owned::PageWrite { addr, data } => {
                    let status = if self.in_session {
                        self.fail_status.unwrap_or_else(|| {
                            let at = usize::from(addr);
                            if self.flash.len() < at + data.len() {
                                self.flash.resize(at + data.len(), 0xFF);
                            }
                            self.flash[at..at + data.len()].copy_from_slice(&data);
                            SessionStatus::Ok
                        })
                    } else {
                        SessionStatus::BadState
                    };
                    let reply_addr = if self.wrong_addr {
                        Some(addr.wrapping_add(2))
                    } else {
                        Some(addr)
                    };
                    Self::reply_of(SessionCmd::PageWrite, reply_addr, data.len(), status)
                }
                Owned::End => {
                    self.in_session = false;
                    Self::reply_of(SessionCmd::End, None, 0, SessionStatus::Ok)
                }
            }
        }

        const fn reply_of(
            cmd: SessionCmd,
            addr: Option<u16>,
            len: usize,
            status: SessionStatus,
        ) -> (Logged, Reply<'static>) {
            (Logged { cmd, addr, len }, Reply::Status { status, addr })
        }
    }

    impl embedded_io::ErrorType for Servant {
        type Error = core::convert::Infallible;
    }

    impl embedded_io::Write for Servant {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.written.extend_from_slice(buf);
            for &byte in buf {
                self.on_byte(byte);
            }
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl embedded_io::Read for Servant {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.readable.len());
            buf[..n].copy_from_slice(&self.readable[..n]);
            self.readable.drain(..n);
            Ok(n)
        }
    }

    /// Runs the session to its terminal outcome, failing on a pump budget.
    fn run(driver: &mut SessionDriver, link: &mut Servant, source: &mut Source) -> Progress {
        for _ in 0..10_000 {
            match driver.pump(link, source) {
                Progress::Pending => {}
                terminal => return terminal,
            }
        }
        panic!("the session must terminate");
    }

    /// The command sequence a happy-path session must produce.
    fn expected_log(payload_len: usize) -> Vec<Logged> {
        let mut log = std::vec![Logged {
            cmd: SessionCmd::Begin,
            addr: None,
            len: 0,
        }];
        let mut addr = 0usize;
        while addr < payload_len {
            let n = PAGE_MAX.min(payload_len - addr);
            log.push(Logged {
                cmd: SessionCmd::PageWrite,
                addr: Some(u16::try_from(addr).unwrap()),
                len: n,
            });
            addr += n;
        }
        log.push(Logged {
            cmd: SessionCmd::End,
            addr: None,
            len: 0,
        });
        log
    }

    #[test]
    fn cellagent_and_cellprog_regions_map_to_targets() {
        assert_eq!(
            target_for(Region::CellagentApp),
            Some(SessionTarget::Cellagent)
        );
        assert_eq!(
            target_for(Region::CellprogApp),
            Some(SessionTarget::CellprogSelf)
        );
        assert_eq!(target_for(Region::ApplicationCode), None);
        assert_eq!(target_for(Region::Bootloader), None);
        assert_eq!(target_for(Region::Factory), None);
    }

    #[test]
    fn happy_path_streams_the_staged_image() {
        // 200 bytes: three full pages plus a short fourth.
        let payload: Vec<u8> = (0..200u32)
            .map(|i| u8::try_from(i % 251).unwrap())
            .collect();
        let mut source = Source::new(&payload);
        let mut link = Servant::new();
        let mut driver = SessionDriver::new();

        driver
            .start(SessionTarget::Cellagent, Region::CellagentApp, &mut source)
            .expect("source is clean");
        assert_eq!(run(&mut driver, &mut link, &mut source), Progress::Success);

        assert_eq!(link.flash, payload);
        assert_eq!(link.log, expected_log(200));
        assert!(!link.in_session);
        assert!(driver.idle());
    }

    #[test]
    fn empty_payload_runs_begin_and_end_only() {
        let mut source = Source::new(&[]);
        let mut link = Servant::new();
        let mut driver = SessionDriver::new();

        driver
            .start(SessionTarget::Cellagent, Region::CellagentApp, &mut source)
            .expect("source is clean");
        assert_eq!(run(&mut driver, &mut link, &mut source), Progress::Success);

        assert!(link.flash.is_empty());
        assert_eq!(link.log, expected_log(0));
    }

    #[test]
    fn corrupt_source_fails_before_any_command() {
        let payload = [7u8; 130];
        let mut source = Source::new(&payload);
        if let Some(byte) = source.image.get_mut(64 + 10) {
            *byte ^= 0x01;
        }
        let link = Servant::new();
        let mut driver = SessionDriver::new();

        assert_eq!(
            driver.start(SessionTarget::Cellagent, Region::CellagentApp, &mut source),
            Err(SessionFailure::CorruptSource)
        );
        assert!(link.written.is_empty());
        assert!(driver.idle());
    }

    #[test]
    fn unreadable_source_fails_before_any_command() {
        let payload = [1u8, 2, 3];
        let mut source = Source::new(&payload);
        source.fail_after = Some(0);
        let mut driver = SessionDriver::new();

        assert_eq!(
            driver.start(SessionTarget::Cellagent, Region::CellagentApp, &mut source),
            Err(SessionFailure::CorruptSource)
        );
    }

    #[test]
    fn store_failure_mid_session_fails_the_session() {
        let payload = [5u8; 200];
        let mut source = Source::new(&payload);
        // The header read plus four CRC passes succeed, the first page load
        // fails.
        source.fail_after = Some(5);
        let mut link = Servant::new();
        let mut driver = SessionDriver::new();

        driver
            .start(SessionTarget::Cellagent, Region::CellagentApp, &mut source)
            .expect("source is clean");
        assert_eq!(
            run(&mut driver, &mut link, &mut source),
            Progress::Failed(SessionFailure::CorruptSource)
        );
    }

    #[test]
    fn oversized_payload_is_rejected() {
        let mut source = Source::new(&[1, 2, 3]);
        // Patch the header to a length beyond u16. The size check must fire
        // before the CRC pass reads anything.
        let mut header = [0u8; 64];
        header.copy_from_slice(&source.image[..64]);
        let mut oversized = ImageHeader::parse(&header).unwrap();
        oversized.payload_len = u32::from(u16::MAX) + 1;
        source.image[..64].copy_from_slice(&oversized.serialize());

        let mut driver = SessionDriver::new();
        assert_eq!(
            driver.start(SessionTarget::Cellagent, Region::CellagentApp, &mut source),
            Err(SessionFailure::Rejected(SessionStatus::InvalidAddr))
        );
    }

    #[test]
    fn silent_link_times_out_after_the_attempt_budget() {
        let payload = [9u8; 64];
        let mut source = Source::new(&payload);
        let mut link = Servant::new();
        link.drop_replies = usize::MAX;
        let mut driver = SessionDriver::new();

        driver
            .start(SessionTarget::Cellagent, Region::CellagentApp, &mut source)
            .expect("source is clean");

        let mut pumps = 0usize;
        let outcome = loop {
            pumps += 1;
            match driver.pump(&mut link, &mut source) {
                Progress::Pending => {}
                terminal => break terminal,
            }
        };

        assert_eq!(outcome, Progress::Failed(SessionFailure::ReplyTimeout));
        // One send pump plus one wait budget per attempt.
        assert_eq!(
            pumps,
            usize::from(MAX_ATTEMPTS) * usize::from(REPLY_WAIT_PUMPS) + 1
        );
        assert_eq!(
            link.log
                .iter()
                .filter(|l| l.cmd == SessionCmd::Begin)
                .count(),
            usize::from(MAX_ATTEMPTS)
        );
        assert!(driver.idle());
    }

    #[test]
    fn lost_replies_are_retried_and_the_session_recovers() {
        let payload: Vec<u8> = (0..100u32).map(|i| u8::try_from(i).unwrap()).collect();
        let mut source = Source::new(&payload);
        let mut link = Servant::new();
        // The first two replies are lost. Both force an idempotent resend,
        // and the third attempt completes the command, so the session still
        // finishes: 4 commands plus 2 resends.
        link.drop_replies = 2;
        let mut driver = SessionDriver::new();

        driver
            .start(SessionTarget::Cellagent, Region::CellagentApp, &mut source)
            .expect("source is clean");
        assert_eq!(run(&mut driver, &mut link, &mut source), Progress::Success);

        assert_eq!(link.flash, payload);
        assert_eq!(link.log.len(), expected_log(100).len() + 2);
    }

    #[test]
    fn rejected_status_fails_without_retry() {
        let payload = [3u8; 64];
        let mut source = Source::new(&payload);
        let mut link = Servant::new();
        link.fail_status = Some(SessionStatus::NotAlive);
        let mut driver = SessionDriver::new();

        driver
            .start(SessionTarget::Cellagent, Region::CellagentApp, &mut source)
            .expect("source is clean");
        assert_eq!(
            run(&mut driver, &mut link, &mut source),
            Progress::Failed(SessionFailure::Rejected(SessionStatus::NotAlive))
        );
        // Begin succeeded, the first page was rejected: two commands, no
        // retries.
        assert_eq!(link.log.len(), 2);
    }

    #[test]
    fn bad_state_page_reply_fails_the_session() {
        // A servant that lost its session state rejects every page
        // deterministically, so the session must fail rather than retry.
        let payload = [3u8; 64];
        let mut source = Source::new(&payload);
        let mut link = Servant::new();
        link.fail_status = Some(SessionStatus::BadState);
        let mut driver = SessionDriver::new();

        driver
            .start(SessionTarget::Cellagent, Region::CellagentApp, &mut source)
            .expect("source is clean");
        assert_eq!(
            run(&mut driver, &mut link, &mut source),
            Progress::Failed(SessionFailure::Rejected(SessionStatus::BadState))
        );
        assert_eq!(link.log.len(), 2);
        assert!(link.flash.is_empty());
    }

    #[test]
    fn mismatched_reply_address_is_retried() {
        let payload = [4u8; 10];
        let mut source = Source::new(&payload);
        let mut link = Servant::new();
        link.wrong_addr = true;
        let mut driver = SessionDriver::new();

        driver
            .start(SessionTarget::Cellagent, Region::CellagentApp, &mut source)
            .expect("source is clean");
        assert_eq!(
            run(&mut driver, &mut link, &mut source),
            Progress::Failed(SessionFailure::ReplyTimeout)
        );
    }

    #[test]
    fn first_written_frame_decodes_as_begin() {
        // The servant mock decodes real frames, so a completed session
        // already proves the encoder. This pins the first frame explicitly.
        let payload = [0xAB, 0xCD, 0xEF, 0x01];
        let mut source = Source::new(&payload);
        let mut link = Servant::new();
        let mut driver = SessionDriver::new();
        driver
            .start(SessionTarget::Cellagent, Region::CellagentApp, &mut source)
            .expect("source is clean");
        assert_eq!(run(&mut driver, &mut link, &mut source), Progress::Success);

        let mut scratch = [0u8; 96];
        let mut decoder = Decoder::new();
        let mut frame = None;
        for &byte in &link.written {
            if let Ok(Some(n)) = decoder.feed(byte, &mut scratch) {
                frame = Some(n);
                break;
            }
        }
        let n = frame.expect("at least one frame was written");
        assert_eq!(
            decode_command(&scratch[..n]).unwrap(),
            WireCommand::Begin(SessionTarget::Cellagent)
        );
    }
}
