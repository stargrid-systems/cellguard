//! Drives the `cellcore` update agent over a concrete byte transport.
//!
//! [`CoreRuntime`] ties the hardware-independent [`Dispatcher`] to three
//! `embedded_io` links: the field bus, the `cellprog` programmer, and an
//! optional downstream node. It forwards agent-bound bus frames and relays
//! replies, answers node-local kinds through a [`TelemetryHandler`], and
//! flashes committed cellagent images through the programmer session. A
//! silent agent gets a `Nack(RouteTimeout)` so the host exchange always
//! completes.
//!
//! The session advances one step per service call, so a multi-second flash
//! never stalls the event loop. The staged image is consumed before the
//! session's first command (see [`Dispatcher::take_pending_program`]), so a
//! reset mid-session cannot re-trigger the same flash on reboot.
//! Application and bootloader images stay staged for their owners: the
//! bootloader self-programs an application image, and a bootloader image is
//! bench-only. A session failure is recorded through
//! `UpdateAgent::record_program_failure`. A bus transfer that interleaves
//! with a running session can overwrite the staged bytes mid-stream, but
//! the only writer is the host that just committed the image.
//!
//! The programmer and agent links must have a receive timeout, because the
//! runtime polls them one bounded read at a time.

#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

use cellboot::io::{ImageStore, KeyStore, StateStore};
use cellcore::update::command::NackReason;
use cellcore::update::dispatch::Dispatcher;
use cellcore::update::session_driver::{Progress, SessionDriver, target_for};
use cellguard_protocol::{Decoder, Header, Kind, Packet};
use embedded_io::{Read, Write};

/// Receive budget for one forwarded agent frame. Agent-bound commands are
/// small, so this only bounds noise.
const AGENT_RX: usize = 64;

/// Bounded reads spent waiting for an agent reply before `RouteTimeout`.
/// One read is one agent-link receive timeout, so 40 reads of 2 ms bound
/// how long a dead node can stall the bus.
const AGENT_REPLY_BUDGET: usize = 40;

/// Payload byte of the `RouteTimeout` nack.
const ROUTE_TIMEOUT_PAYLOAD: [u8; 1] = [NackReason::RouteTimeout.to_code()];

/// Largest telemetry reply payload this runtime can build.
const TELEMETRY_PAYLOAD_MAX: usize = 17;

enum RouteAction {
    Forward(usize),
    Telemetry(Kind, usize),
}

/// A side handler for node-local request kinds the update agent does not
/// own.
pub trait TelemetryHandler {
    /// Serves one request.
    ///
    /// Returns `None` to leave the frame unanswered.
    fn handle(
        &mut self,
        now: u32,
        kind: Kind,
        payload: &[u8],
        out: &mut [u8],
    ) -> Option<(Kind, usize)>;

    /// Observes a frame after it was written to the agent link.
    fn note_forwarded(&mut self, _kind: Kind, _payload: &[u8]) {}

    /// Called once per [`CoreRuntime::tick`].
    fn on_tick(&mut self, _now: u32) {}
}

/// Hosts the update agent on its byte links.
///
/// `Bus` is the field bus, `Prog` and `Agent` reach the programmer and the
/// downstream node, and `RX` sizes the dispatcher receive buffer.
pub struct CoreRuntime<'k, S, K, St, Bus, Prog, Agent, const RX: usize> {
    dispatcher: Dispatcher<'k, S, K, St, RX>,
    bus: Bus,
    prog: Prog,
    agent: Agent,
    agent_id: u8,
    node_id: u8,
    /// Guards that `confirm_app_healthy` is persisted at most once per boot.
    app_confirmed: bool,
    /// Idle unless a cellagent flash is in flight.
    session: SessionDriver,
    route_decoder: Decoder,
    route_scratch: [u8; AGENT_RX],
    reply_decoder: Decoder,
    telemetry: Option<&'k mut dyn TelemetryHandler>,
    now: u32,
}

impl<'k, S, K, St, Bus, Prog, Agent, const RX: usize>
    CoreRuntime<'k, S, K, St, Bus, Prog, Agent, RX>
where
    S: ImageStore,
    K: KeyStore,
    St: StateStore,
    Bus: Read + Write,
    Prog: Read + Write,
    Agent: Read + Write,
{
    /// Creates a runtime around `dispatcher`.
    ///
    /// Bus frames addressed to `agent_id` are routed over `agent`.
    pub const fn new(
        dispatcher: Dispatcher<'k, S, K, St, RX>,
        bus: Bus,
        prog: Prog,
        agent: Agent,
        agent_id: u8,
    ) -> Self {
        Self {
            dispatcher,
            bus,
            prog,
            agent,
            agent_id,
            node_id: 0,
            app_confirmed: false,
            session: SessionDriver::new(),
            route_decoder: Decoder::new(),
            route_scratch: [0; AGENT_RX],
            reply_decoder: Decoder::new(),
            telemetry: None,
            now: 0,
        }
    }

    /// Installs a telemetry handler and this node's bus address. The
    /// handler answers the kinds it owns, the update agent answers its own.
    #[must_use]
    pub fn with_telemetry(mut self, handler: &'k mut dyn TelemetryHandler, node_id: u8) -> Self {
        self.telemetry = Some(handler);
        self.node_id = node_id;
        self
    }

    /// Advances the runtime's tick, passed to telemetry handlers to stamp
    /// their refresh windows.
    pub fn tick(&mut self, now: u32) {
        self.now = now;
        if let Some(handler) = self.telemetry.as_deref_mut() {
            handler.on_tick(now);
        }
    }

    /// Runs the agent forever.
    ///
    /// An application with other duties can call [`CoreRuntime::try_service`]
    /// from its own event loop instead.
    pub fn run(&mut self) -> ! {
        loop {
            self.try_service();
        }
    }

    /// Attempts to read and service one bus byte.
    ///
    /// Returns immediately when no byte arrives within the bus receive
    /// timeout, after advancing an in-flight programming session.
    pub fn try_service(&mut self) {
        let mut buf = [0u8; 1];
        if self.bus.read_exact(&mut buf).is_ok()
            && let Some(&byte) = buf.first()
        {
            self.service(byte);
        } else {
            self.pump_session();
        }
    }

    /// Services one received bus byte.
    ///
    /// The first successful exchange in a boot also marks the running
    /// application healthy. Link errors are swallowed: a dropped bus
    /// response is retried on the next byte.
    pub fn service(&mut self, byte: u8) {
        self.route(byte);
        if let Some(response) = self.dispatcher.feed(byte) {
            let delivered = self.bus.write_all(response).is_ok();
            if delivered && !self.app_confirmed {
                self.dispatcher.agent_mut().confirm_app_healthy();
                self.app_confirmed = true;
            }
        }
        self.pump_session();
    }

    /// Starts a committed cellagent flash and advances an in-flight session
    /// by one step. Regions the programmer link cannot flash stay staged,
    /// and any session failure is recorded as a program failure.
    fn pump_session(&mut self) {
        if self.session.idle()
            && let Some(region) = self.dispatcher.agent().pending_program()
            && let Some(target) = target_for(region)
        {
            let _ = self.dispatcher.take_pending_program();
            if self
                .session
                .start(target, region, self.dispatcher.agent_mut())
                .is_err()
            {
                self.dispatcher.agent_mut().record_program_failure();
            }
        }
        if matches!(
            self.session
                .pump(&mut self.prog, self.dispatcher.agent_mut()),
            Progress::Failed(_)
        ) {
            self.dispatcher.agent_mut().record_program_failure();
        }
    }

    /// Watches bus bytes for agent-bound frames and forwards them. The
    /// dispatcher ignores foreign-id frames, so the same byte safely feeds
    /// both decoders.
    fn route(&mut self, byte: u8) {
        let mut frame_buf = [0u8; AGENT_RX];
        let action = {
            let Ok(Some(len)) = self.route_decoder.feed(byte, &mut self.route_scratch) else {
                return;
            };
            let Some(frame) = self.route_scratch.get(..len) else {
                return;
            };
            let Ok((header, _)) = Header::parse(frame) else {
                return;
            };
            if header.id == self.agent_id {
                let Some(slot) = frame_buf.get_mut(..len) else {
                    return;
                };
                slot.copy_from_slice(frame);
                RouteAction::Forward(len)
            } else if header.id == self.node_id {
                // A frame that does not parse is not ours to answer.
                match Packet::parse(frame) {
                    Ok(packet) => {
                        let Some(slot) = frame_buf.get_mut(..len) else {
                            return;
                        };
                        slot.copy_from_slice(frame);
                        RouteAction::Telemetry(packet.kind, len)
                    }
                    Err(_) => return,
                }
            } else {
                return;
            }
        };
        match action {
            RouteAction::Forward(len) => {
                let Some(frame) = frame_buf.get(..len) else {
                    return;
                };
                let mut payload_buf = [0u8; AGENT_RX];
                let noted = Packet::parse(frame).ok().map(|packet| {
                    let payload = payload_buf.get_mut(..packet.payload.len())?;
                    payload.copy_from_slice(packet.payload);
                    Some((packet.kind, payload.len()))
                });
                self.forward(frame);
                if let (Some(handler), Some(Some((kind, payload_len)))) =
                    (self.telemetry.as_deref_mut(), noted)
                {
                    handler.note_forwarded(kind, payload_buf.get(..payload_len).unwrap_or(&[]));
                }
            }
            RouteAction::Telemetry(kind, len) => {
                let Some(frame) = frame_buf.get(..len) else {
                    return;
                };
                self.serve_telemetry(kind, frame);
            }
        }
    }

    fn serve_telemetry(&mut self, kind: Kind, frame: &[u8]) {
        let Some(handler) = self.telemetry.as_deref_mut() else {
            return;
        };
        let payload = Packet::parse(frame).map_or(&[] as &[u8], |packet| packet.payload);
        let mut out = [0u8; TELEMETRY_PAYLOAD_MAX];
        let Some((kind, len)) = handler.handle(self.now, kind, payload, &mut out) else {
            return;
        };
        let mut raw = [0u8; TELEMETRY_PAYLOAD_MAX
            + cellguard_protocol::HEADER_LEN
            + cellguard_protocol::PAYLOAD_CRC_LEN];
        if let Ok(raw_len) =
            Packet::write(self.node_id, kind, out.get(..len).unwrap_or(&[]), &mut raw)
            && let Some(raw_slice) = raw.get(..raw_len)
        {
            let mut wire = [0u8; cellguard_protocol::max_encoded_len(
                TELEMETRY_PAYLOAD_MAX
                    + cellguard_protocol::HEADER_LEN
                    + cellguard_protocol::PAYLOAD_CRC_LEN,
            )];
            if let Some(wire_len) = cellguard_protocol::encode_frame(raw_slice, &mut wire)
                && let Some(bytes) = wire.get(..wire_len)
            {
                let _ = self.bus.write_all(bytes);
            }
        }
    }

    fn forward(&mut self, frame: &[u8]) {
        let mut wire = [0u8; cellguard_protocol::max_encoded_len(AGENT_RX)];
        let Some(len) = cellguard_protocol::encode_frame(frame, &mut wire) else {
            return;
        };
        if self
            .agent
            .write_all(wire.get(..len).unwrap_or(&[]))
            .is_err()
        {
            self.send_route_timeout();
            return;
        }

        let mut scratch = [0u8; AGENT_RX];
        self.reply_decoder = Decoder::new();
        for _ in 0..AGENT_REPLY_BUDGET {
            let mut byte = [0u8; 1];
            if self.agent.read_exact(&mut byte).is_err() {
                continue;
            }
            if let Ok(Some(n)) = self.reply_decoder.feed(byte[0], &mut scratch)
                && let Some(reply) = scratch.get(..n)
            {
                let mut reply_wire = [0u8; cellguard_protocol::max_encoded_len(AGENT_RX)];
                if let Some(reply_len) = cellguard_protocol::encode_frame(reply, &mut reply_wire)
                    && let Some(bytes) = reply_wire.get(..reply_len)
                {
                    let _ = self.bus.write_all(bytes);
                }
                return;
            }
        }
        self.send_route_timeout();
    }

    fn send_route_timeout(&mut self) {
        let mut raw =
            [0u8; cellguard_protocol::HEADER_LEN + 1 + cellguard_protocol::PAYLOAD_CRC_LEN];
        if let Ok(len) = Packet::write(
            self.agent_id,
            cellguard_protocol::Kind::Nack,
            &ROUTE_TIMEOUT_PAYLOAD,
            &mut raw,
        ) && let Some(raw_slice) = raw.get(..len)
        {
            let mut wire = [0u8; cellguard_protocol::max_encoded_len(
                cellguard_protocol::HEADER_LEN + 1 + cellguard_protocol::PAYLOAD_CRC_LEN,
            )];
            if let Some(wire_len) = cellguard_protocol::encode_frame(raw_slice, &mut wire)
                && let Some(bytes) = wire.get(..wire_len)
            {
                let _ = self.bus.write_all(bytes);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use core::cell::RefCell;

    use cellboot::image::{ImageHeader, ImageKind, Region};
    use cellboot::io::NoKeyStore;
    use cellboot::state::{AppHealth, PersistentState, StagedState, UpdateOutcome};
    use cellboot::testutil::{NullStateStore, SharedImageStore};
    use cellcore::update::dispatch::Dispatcher;
    use cellcore::update::session::{RegionSlot, StagingLayout, UpdateAgent};
    use cellguard_protocol::{
        Command as WireCommand, Decoder, Encoder, Kind, Packet, Reply, SessionCmd, SessionStatus,
        decode_command, encode_frame, encode_reply,
    };

    use super::CoreRuntime;

    const KEY: [u8; 16] = *b"runtime-test-key";
    const TARGET: u16 = 0x33;
    const CELLAGENT_TARGET: u16 = 0x34;
    const NODE: u8 = 7;
    const AGENT_ID: u8 = 9;
    const CAP: usize = 4096;
    const HEADER_LEN: usize = 64;

    /// The runtime under test: a shared backing store so tests can stage and
    /// corrupt images while the agent owns the store, plus a bus mock, a
    /// programmer-servant mock, and an agent-link mock.
    type Runtime<'k, 'a> = CoreRuntime<
        'k,
        SharedImageStore<'a, CAP>,
        NoKeyStore,
        NullStateStore,
        MockLink,
        ServantLink,
        MockLink,
        512,
    >;

    /// A link that records everything written and yields scripted read bytes.
    #[derive(Default)]
    struct MockLink {
        written: std::vec::Vec<u8>,
        readable: std::vec::Vec<u8>,
    }

    impl MockLink {
        /// Encodes a packet as a COBS frame and queues it for reading.
        fn queue_packet(&mut self, id: u8, kind: Kind, payload: &[u8]) {
            let mut raw = [0u8; 64];
            let n = Packet::write(id, kind, payload, &mut raw).unwrap();
            let mut encoder = Encoder::new(&raw[..n]);
            while let Some(byte) = encoder.pull() {
                self.readable.push(byte);
            }
        }
    }

    impl embedded_io::ErrorType for MockLink {
        type Error = core::convert::Infallible;
    }

    impl embedded_io::Write for MockLink {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.written.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    impl embedded_io::Read for MockLink {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.readable.len());
            buf[..n].copy_from_slice(&self.readable[..n]);
            self.readable.drain(..n);
            Ok(n)
        }
    }

    /// A decoded command, copied out of the decode buffer so the mock can
    /// mutate itself while executing it.
    enum Owned {
        Begin,
        PageWrite { addr: u16, data: std::vec::Vec<u8> },
        End,
    }

    /// A programmer-servant mock: decodes written commands with the real
    /// codec, runs a minimal servant against a fake flash, and queues real
    /// reply frames.
    struct ServantLink {
        written: std::vec::Vec<u8>,
        readable: std::vec::Vec<u8>,
        decoder: Decoder,
        scratch: [u8; 96],
        in_session: bool,
        flash: std::vec::Vec<u8>,
        commands: std::vec::Vec<SessionCmd>,
        /// Status to reply to page commands instead of writing flash.
        fail_status: Option<SessionStatus>,
        /// Swallow all replies, simulating a dead link.
        silent: bool,
    }

    impl ServantLink {
        fn new() -> Self {
            Self {
                written: std::vec::Vec::new(),
                readable: std::vec::Vec::new(),
                decoder: Decoder::new(),
                scratch: [0; 96],
                in_session: false,
                flash: std::vec::Vec::new(),
                commands: std::vec::Vec::new(),
                fail_status: None,
                silent: false,
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
                _ => return,
            };
            let reply = self.execute(owned);
            if self.silent {
                return;
            }
            let mut raw = [0u8; 1 + 3 + cellguard_protocol::PAGE_MAX + 2];
            let n = encode_reply(reply, &mut raw).unwrap();
            let mut wire = [0u8; 96];
            let wire_len = encode_frame(&raw[..n], &mut wire).unwrap();
            self.readable.extend_from_slice(&wire[..wire_len]);
        }

        /// Runs one decoded command against the fake flash and returns its
        /// reply.
        fn execute(&mut self, cmd: Owned) -> Reply<'static> {
            match cmd {
                Owned::Begin => {
                    self.in_session = true;
                    self.commands.push(SessionCmd::Begin);
                    Reply::Status {
                        status: SessionStatus::Ok,
                        addr: None,
                    }
                }
                Owned::PageWrite { addr, data } => {
                    self.commands.push(SessionCmd::PageWrite);
                    let status = if !self.in_session {
                        SessionStatus::BadState
                    } else if let Some(status) = self.fail_status {
                        status
                    } else {
                        let at = usize::from(addr);
                        if self.flash.len() < at + data.len() {
                            self.flash.resize(at + data.len(), 0xFF);
                        }
                        self.flash[at..at + data.len()].copy_from_slice(&data);
                        SessionStatus::Ok
                    };
                    Reply::Status {
                        status,
                        addr: Some(addr),
                    }
                }
                Owned::End => {
                    self.in_session = false;
                    self.commands.push(SessionCmd::End);
                    Reply::Status {
                        status: SessionStatus::Ok,
                        addr: None,
                    }
                }
            }
        }
    }

    impl embedded_io::ErrorType for ServantLink {
        type Error = core::convert::Infallible;
    }

    impl embedded_io::Write for ServantLink {
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

    impl embedded_io::Read for ServantLink {
        fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
            let n = buf.len().min(self.readable.len());
            buf[..n].copy_from_slice(&self.readable[..n]);
            self.readable.drain(..n);
            Ok(n)
        }
    }

    fn layout() -> StagingLayout {
        StagingLayout {
            application: RegionSlot {
                offset: 0,
                capacity: 2048,
            },
            bootloader: RegionSlot {
                offset: 2048,
                capacity: 2048,
            },
            cellagent: RegionSlot {
                offset: 3072,
                capacity: 1024,
            },
        }
    }

    /// Writes an image (header plus payload) into the region's slot of the
    /// shared backing store, as a commit would have.
    fn stage_image(backing: &RefCell<[u8; CAP]>, region: Region, payload: &[u8]) {
        let header = ImageHeader {
            kind: ImageKind::Application,
            region,
            target_id: TARGET,
            fw_version: 9,
            payload_len: u32::try_from(payload.len()).unwrap(),
            payload_crc32: crc::checksum32(payload),
            hmac: [0u8; 32],
        };
        let offset = match region {
            Region::ApplicationCode => 0,
            Region::Bootloader => 2048,
            Region::CellagentApp => 3072,
            _ => panic!("not a firmware slot"),
        };
        let mut image = header.serialize().to_vec();
        image.extend_from_slice(payload);
        let mut backing = backing.borrow_mut();
        backing[offset..offset + image.len()].copy_from_slice(&image);
    }

    fn runtime_with<'k, 'a>(
        key: &'k mut [u8; 16],
        state: PersistentState,
        backing: &'a RefCell<[u8; CAP]>,
    ) -> Runtime<'k, 'a> {
        let agent = UpdateAgent::new(
            SharedImageStore::new(backing),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            key,
            NoKeyStore,
            NullStateStore,
            state,
        );
        let dispatcher = Dispatcher::new(agent, NODE);
        CoreRuntime::new(
            dispatcher,
            MockLink::default(),
            ServantLink::new(),
            MockLink::default(),
            AGENT_ID,
        )
    }

    /// Feeds a COBS-encoded frame addressed to `id` byte by byte.
    fn feed_from(runtime: &mut Runtime<'_, '_>, id: u8, kind: Kind, payload: &[u8]) {
        let mut raw = [0u8; 64];
        let raw_len = Packet::write(id, kind, payload, &mut raw).unwrap();
        let mut encoder = Encoder::new(&raw[..raw_len]);
        while let Some(byte) = encoder.pull() {
            runtime.service(byte);
        }
    }

    /// Feeds a COBS-encoded command frame byte by byte.
    fn feed_command(runtime: &mut Runtime<'_, '_>, kind: Kind, payload: &[u8]) {
        feed_from(runtime, NODE, kind, payload);
    }

    fn decode_kind(frame: &[u8]) -> Kind {
        let mut scratch = [0u8; 128];
        let mut decoder = Decoder::new();
        let mut done = None;
        for &byte in frame {
            if let Some(n) = decoder.feed(byte, &mut scratch).unwrap() {
                done = Some(n);
            }
        }
        let n = done.expect("response frame did not complete");
        Packet::parse(&scratch[..n]).unwrap().kind
    }

    /// Drives the runtime until its programming session goes idle again.
    fn run_session(runtime: &mut Runtime<'_, '_>) {
        for _ in 0..2_000 {
            runtime.try_service();
        }
    }

    fn ready_state(region: Region) -> PersistentState {
        PersistentState {
            agent_version: 1,
            app_version: 0,
            staged_version: 9,
            app_health: AppHealth::Unknown,
            staged: StagedState::Ready,
            staged_region: Some(region),
            last_outcome: UpdateOutcome::None,
            program_attempts: 0,
            boot_count: 0,
        }
    }

    #[test]
    fn probe_is_answered_on_the_bus_only() {
        let backing = RefCell::new([0u8; CAP]);
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, PersistentState::new(1), &backing);
        feed_command(&mut runtime, Kind::BootProbe, &[]);

        assert_eq!(decode_kind(&runtime.bus.written), Kind::BootStatus);
        assert!(
            runtime.prog.written.is_empty(),
            "a probe must not start a session"
        );
    }

    #[test]
    fn first_successful_exchange_marks_app_healthy() {
        let backing = RefCell::new([0u8; CAP]);
        let mut state = PersistentState::new(1);
        state.boot_count = 3;
        state.app_health = AppHealth::Unknown;
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, state, &backing);

        assert_eq!(
            runtime.dispatcher.agent().status().app_health,
            AppHealth::Unknown
        );
        feed_command(&mut runtime, Kind::BootProbe, &[]);
        assert_eq!(
            runtime.dispatcher.agent().status().app_health,
            AppHealth::Good
        );
        assert_eq!(runtime.dispatcher.agent().status().boot_count, 0);
    }

    #[test]
    fn committed_cellagent_image_is_flashed_over_one_session() {
        let payload: std::vec::Vec<u8> = (0..150u32).map(|i| u8::try_from(i).unwrap()).collect();
        let backing = RefCell::new([0u8; CAP]);
        stage_image(&backing, Region::CellagentApp, &payload);
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, ready_state(Region::CellagentApp), &backing);

        run_session(&mut runtime);

        assert_eq!(
            runtime.prog.flash, payload,
            "the servant received the image"
        );
        assert_eq!(
            runtime.prog.commands.first(),
            Some(&SessionCmd::Begin),
            "the session must open with Begin"
        );
        assert_eq!(
            runtime.prog.commands.last(),
            Some(&SessionCmd::End),
            "the session must close with End"
        );
        assert_eq!(
            runtime.dispatcher.agent().status().staged,
            StagedState::Empty,
            "the image is consumed"
        );
        assert_eq!(
            runtime.dispatcher.agent().status().last_outcome,
            UpdateOutcome::Success
        );

        let sent = runtime.prog.commands.len();
        runtime.service(0);
        run_session(&mut runtime);
        assert_eq!(runtime.prog.commands.len(), sent);
    }

    #[test]
    fn committed_app_and_bootloader_images_stay_staged() {
        for region in [Region::ApplicationCode, Region::Bootloader] {
            let backing = RefCell::new([0u8; CAP]);
            stage_image(&backing, region, &[1, 2, 3]);
            let mut key = KEY;
            let mut runtime = runtime_with(&mut key, ready_state(region), &backing);

            runtime.service(0);
            run_session(&mut runtime);

            assert!(
                runtime.prog.written.is_empty(),
                "{region:?} must not be flashed over the programmer link"
            );
            assert_eq!(
                runtime.dispatcher.agent().status().staged,
                StagedState::Ready,
                "{region:?} stays staged for its owner"
            );
        }
    }

    #[test]
    fn rejected_session_records_program_failed() {
        let backing = RefCell::new([0u8; CAP]);
        stage_image(&backing, Region::CellagentApp, &[1, 2, 3, 4]);
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, ready_state(Region::CellagentApp), &backing);
        runtime.prog.fail_status = Some(SessionStatus::NotAlive);

        run_session(&mut runtime);

        assert_eq!(
            runtime.dispatcher.agent().status().last_outcome,
            UpdateOutcome::ProgramFailed
        );
        assert_eq!(
            runtime.dispatcher.agent().status().staged,
            StagedState::Empty
        );
    }

    #[test]
    fn silent_servant_records_program_failed() {
        let backing = RefCell::new([0u8; CAP]);
        stage_image(&backing, Region::CellagentApp, &[1, 2, 3, 4]);
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, ready_state(Region::CellagentApp), &backing);
        runtime.prog.silent = true;

        run_session(&mut runtime);

        assert_eq!(
            runtime.dispatcher.agent().status().last_outcome,
            UpdateOutcome::ProgramFailed
        );
    }

    #[test]
    fn corrupt_staged_image_never_signals_the_programmer() {
        let backing = RefCell::new([0u8; CAP]);
        stage_image(&backing, Region::CellagentApp, &[7u8; 100]);
        backing.borrow_mut()[3072 + HEADER_LEN + 5] ^= 0x01;
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, ready_state(Region::CellagentApp), &backing);

        runtime.service(0);
        run_session(&mut runtime);

        assert!(
            runtime.prog.written.is_empty(),
            "a corrupt source must never reach the servant"
        );
        assert_eq!(
            runtime.dispatcher.agent().status().last_outcome,
            UpdateOutcome::ProgramFailed
        );
        assert_eq!(
            runtime.dispatcher.agent().status().staged,
            StagedState::Empty
        );
    }

    #[test]
    fn agent_bound_frame_is_forwarded_and_reply_relayed() {
        let backing = RefCell::new([0u8; CAP]);
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, PersistentState::new(1), &backing);
        runtime
            .agent
            .queue_packet(AGENT_ID, Kind::BalancerGateState, &[0x03]);

        feed_from(&mut runtime, AGENT_ID, Kind::ReadBalancerGateState, &[]);

        assert_eq!(
            decode_kind(&runtime.agent.written),
            Kind::ReadBalancerGateState
        );
        assert_eq!(decode_kind(&runtime.bus.written), Kind::BalancerGateState);
    }

    #[test]
    fn silent_agent_gets_a_route_timeout_nack() {
        let backing = RefCell::new([0u8; CAP]);
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, PersistentState::new(1), &backing);

        feed_from(&mut runtime, AGENT_ID, Kind::SetBalancer, &[0x01]);

        assert_eq!(decode_kind(&runtime.bus.written), Kind::Nack);
    }

    #[test]
    fn own_frames_still_answer_and_do_not_touch_the_agent_link() {
        let backing = RefCell::new([0u8; CAP]);
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, PersistentState::new(1), &backing);

        feed_command(&mut runtime, Kind::BootProbe, &[]);

        assert_eq!(decode_kind(&runtime.bus.written), Kind::BootStatus);
        assert!(runtime.agent.written.is_empty());
    }

    /// A telemetry handler that answers `ReadRails` and records forwarded
    /// kinds.
    struct EchoHandler {
        noted: std::vec::Vec<(Kind, u8)>,
    }

    impl super::TelemetryHandler for EchoHandler {
        fn handle(
            &mut self,
            _now: u32,
            kind: Kind,
            _payload: &[u8],
            out: &mut [u8],
        ) -> Option<(Kind, usize)> {
            match kind {
                Kind::ReadRails => {
                    out[0] = 1;
                    out[1] = 2;
                    Some((Kind::Rails, 2))
                }
                _ => None,
            }
        }

        fn note_forwarded(&mut self, kind: Kind, payload: &[u8]) {
            if let Some(&mask) = payload.first() {
                self.noted.push((kind, mask));
            }
        }
    }

    #[test]
    fn telemetry_kinds_are_answered_by_the_side_handler() {
        let backing = RefCell::new([0u8; CAP]);
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1), &backing);
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
        };
        let runtime = &mut runtime.with_telemetry(&mut handler, NODE);
        feed_command(runtime, Kind::ReadRails, &[]);
        assert_eq!(decode_kind(&runtime.bus.written), Kind::Rails);
    }

    #[test]
    fn boot_kinds_still_reach_the_update_agent_alongside_telemetry() {
        let backing = RefCell::new([0u8; CAP]);
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1), &backing);
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
        };
        let runtime = &mut runtime.with_telemetry(&mut handler, NODE);
        feed_command(runtime, Kind::BootProbe, &[]);
        assert_eq!(decode_kind(&runtime.bus.written), Kind::BootStatus);
    }

    #[test]
    fn forwarded_set_balancer_notifies_the_handler() {
        let backing = RefCell::new([0u8; CAP]);
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1), &backing);
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
        };
        {
            let mut runtime = runtime.with_telemetry(&mut handler, NODE);
            feed_from(&mut runtime, AGENT_ID, Kind::SetBalancer, &[0x03]);
        }
        assert_eq!(handler.noted, std::vec![(Kind::SetBalancer, 0x03)]);
    }
}
