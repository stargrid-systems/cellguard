//! Drives the `cellcore` update agent over a concrete byte transport.
//!
//! [`CoreRuntime`] ties the hardware-independent [`Dispatcher`] to three
//! `embedded_io` links: the field bus, the `cellprog` programmer, and an
//! optional downstream node. It forwards agent-bound bus frames and relays
//! replies, answers node-local kinds through a [`TelemetryHandler`], and
//! Hands committed images to the programmer. A silent agent gets a
//! `Nack(RouteTimeout)` so the host exchange always completes.
//!
//! [`CoreRuntime::try_poll_agent_temp`] polls the downstream node's
//! temperature on a slow cadence and reports it through the telemetry
//! handler. Like a forward it is bounded, but it stays off the bus.
//!
//! Handoff is consume-before-signal: [`Dispatcher::take_pending_program`]
//! clears the staged image before the programmer is signaled, so a reset
//! mid-programming cannot re-trigger the same flash on reboot. The cost is
//! a lost update if the programmer-link write fails. The programmer and
//! agent links must have a receive timeout, because the runtime polls them
//! one bounded read at a time.

#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

use cellboot::image::Region;
use cellboot::io::{ImageStore, KeyStore, StateStore};
use cellcore::update::command::NackReason;
use cellcore::update::dispatch::Dispatcher;
use cellcore::update::handoff::{self, PROGRAM_WIRE, RESULT_FRAME};
use cellguard_protocol::{Decoder, Header, Kind, Packet, ProgStatus};
use embedded_io::{Read, Write};

/// Receive budget for one forwarded agent frame. Agent-bound commands are
/// small, so this only bounds noise.
const AGENT_RX: usize = 64;

/// Bounded reads spent waiting for an agent reply before `RouteTimeout`.
/// One read is one agent-link receive timeout, so 40 reads of 2 ms bound
/// how long a dead node can stall the bus.
const AGENT_REPLY_BUDGET: usize = 40;

/// Routed temperature poll cadence in ticks (about 1 s at 1.024 kHz).
const AGENT_TEMP_POLL_TICKS: u32 = 1024;

/// Raw length of the poll request: header, empty payload, payload CRC.
const AGENT_TEMP_REQ_RAW: usize =
    cellguard_protocol::HEADER_LEN + cellguard_protocol::PAYLOAD_CRC_LEN;

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

    /// Outcome of one routed temperature poll: centi-degrees Celsius, or
    /// `None` for a missed reply.
    fn note_agent_temp(&mut self, _temp: Option<i16>) {}

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
    prog_id: u8,
    node_id: u8,
    /// Guards that `confirm_app_healthy` is persisted at most once per boot.
    app_confirmed: bool,
    /// Set after a handoff frame is written, cleared when a `ProgResult`
    /// reply arrives. While set, each serviced byte polls the programmer
    /// link.
    awaiting_prog_result: bool,
    prog_decoder: Decoder,
    prog_scratch: [u8; RESULT_FRAME],
    route_decoder: Decoder,
    route_scratch: [u8; AGENT_RX],
    reply_decoder: Decoder,
    last_agent_poll: u32,
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
        prog_id: u8,
        agent: Agent,
        agent_id: u8,
    ) -> Self {
        Self {
            dispatcher,
            bus,
            prog,
            agent,
            agent_id,
            prog_id,
            node_id: 0,
            app_confirmed: false,
            awaiting_prog_result: false,
            prog_decoder: Decoder::new(),
            prog_scratch: [0; RESULT_FRAME],
            route_decoder: Decoder::new(),
            route_scratch: [0; AGENT_RX],
            reply_decoder: Decoder::new(),
            last_agent_poll: 0,
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

    /// Runs one routed [`Kind::ReadTemperature`] poll when the cadence
    /// elapsed and reports the outcome through
    /// [`TelemetryHandler::note_agent_temp`]. No-ops without a telemetry
    /// handler or before one cadence period passed.
    pub fn try_poll_agent_temp(&mut self) {
        if self.telemetry.is_none()
            || self.now.wrapping_sub(self.last_agent_poll) < AGENT_TEMP_POLL_TICKS
        {
            return;
        }
        self.last_agent_poll = self.now;
        let temp = self.poll_agent_temp();
        if let Some(handler) = self.telemetry.as_deref_mut() {
            handler.note_agent_temp(temp);
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
    /// timeout, after polling an in-flight programmer reply.
    pub fn try_service(&mut self) {
        let mut buf = [0u8; 1];
        if self.bus.read_exact(&mut buf).is_ok()
            && let Some(&byte) = buf.first()
        {
            self.service(byte);
        } else if self.awaiting_prog_result {
            self.poll_prog_result();
        }
    }

    /// Services one received bus byte.
    ///
    /// The first successful exchange in a boot also marks the running
    /// application healthy. Link errors are swallowed: a dropped bus
    /// response is retried on the next byte, a dropped handoff is not.
    pub fn service(&mut self, byte: u8) {
        self.route(byte);
        if let Some(response) = self.dispatcher.feed(byte) {
            let delivered = self.bus.write_all(response).is_ok();
            if delivered && !self.app_confirmed {
                self.dispatcher.agent_mut().confirm_app_healthy();
                self.app_confirmed = true;
            }
        }
        if let Some(region) = self.dispatcher.agent().pending_program() {
            self.hand_off(region);
        }
        if self.awaiting_prog_result {
            self.poll_prog_result();
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

    /// Exchanges one `ReadTemperature` request with the downstream node.
    fn poll_agent_temp(&mut self) -> Option<i16> {
        let mut raw = [0u8; AGENT_TEMP_REQ_RAW];
        let len = Packet::write(self.agent_id, Kind::ReadTemperature, &[], &mut raw).ok()?;
        let mut wire = [0u8; cellguard_protocol::max_encoded_len(AGENT_TEMP_REQ_RAW)];
        let wire_len = cellguard_protocol::encode_frame(raw.get(..len)?, &mut wire)?;
        self.agent
            .write_all(wire.get(..wire_len).unwrap_or(&[]))
            .ok()?;

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
                return Packet::parse(reply)
                    .ok()
                    .filter(|packet| packet.kind == Kind::Temperature)
                    .and_then(|packet| <[u8; 2]>::try_from(packet.payload).ok())
                    .map(i16::from_le_bytes);
            }
        }
        None
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

    /// Signals the programmer to flash the committed `region`.
    ///
    /// Consume-before-signal (see the module docs): a failed write to the
    /// link loses the update.
    fn hand_off(&mut self, region: Region) {
        let mut frame = [0u8; PROGRAM_WIRE];
        let Some(len) = handoff::program_frame(self.prog_id, region, &mut frame) else {
            return;
        };
        let _ = self.dispatcher.take_pending_program();
        if let Some(bytes) = frame.get(..len)
            && self.prog.write_all(bytes).is_ok()
        {
            self.awaiting_prog_result = true;
        }
    }

    /// Polls the programmer link for the `ProgResult` reply of an in-flight
    /// handoff, one bounded read per call. A read timeout or link error
    /// leaves the poll armed. A completed frame clears it, and only a
    /// reported failure is recorded.
    fn poll_prog_result(&mut self) {
        let mut byte = [0u8; 1];
        if self.prog.read_exact(&mut byte).is_err() {
            return;
        }
        let Ok(Some(n)) = self.prog_decoder.feed(byte[0], &mut self.prog_scratch) else {
            return;
        };
        self.awaiting_prog_result = false;
        if n == 0 {
            return;
        }
        let status = Packet::parse(self.prog_scratch.get(..n).unwrap_or(&[]))
            .ok()
            .and_then(|packet| handoff::parse_result(&packet));
        if let Some(status) = status
            && status != ProgStatus::Ok
        {
            self.dispatcher.agent_mut().record_program_failure();
        }
    }
}

#[cfg(test)]
mod tests {
    use cellboot::image::Region;
    use cellboot::io::NoKeyStore;
    use cellboot::state::{AppHealth, PersistentState, StagedState, UpdateOutcome};
    use cellboot::testutil::{MemStore as MemStoreImpl, NullStateStore};
    use cellcore::update::dispatch::Dispatcher;
    use cellcore::update::session::{RegionSlot, StagingLayout, UpdateAgent};
    use cellguard_protocol::{Decoder, Encoder, Kind, Packet, ProgStatus};

    use super::CoreRuntime;

    const KEY: [u8; 16] = *b"runtime-test-key";
    const TARGET: u16 = 0x33;
    const CELLAGENT_TARGET: u16 = 0x34;
    const NODE: u8 = 7;
    const PROG_ID: u8 = 4;
    const AGENT_ID: u8 = 9;
    const CAP: usize = 4096;
    type MemStore = MemStoreImpl<CAP>;

    #[derive(Default)]
    struct MockLink {
        written: std::vec::Vec<u8>,
        readable: std::vec::Vec<u8>,
    }

    impl MockLink {
        fn queue_packet(&mut self, kind: Kind, payload: &[u8]) {
            let mut raw = [0u8; 64];
            let n = Packet::write(PROG_ID, kind, payload, &mut raw).unwrap();
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

    fn runtime_with(
        key: &mut [u8; 16],
        state: PersistentState,
    ) -> CoreRuntime<'_, MemStore, NoKeyStore, NullStateStore, MockLink, MockLink, MockLink, 512>
    {
        let agent = UpdateAgent::new(
            MemStore::new(),
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
            MockLink::default(),
            PROG_ID,
            MockLink::default(),
            AGENT_ID,
        )
    }

    fn feed_from(
        runtime: &mut CoreRuntime<
            '_,
            MemStore,
            NoKeyStore,
            NullStateStore,
            MockLink,
            MockLink,
            MockLink,
            512,
        >,
        id: u8,
        kind: Kind,
        payload: &[u8],
    ) {
        let mut raw = [0u8; 64];
        let raw_len = Packet::write(id, kind, payload, &mut raw).unwrap();
        let mut encoder = Encoder::new(&raw[..raw_len]);
        while let Some(byte) = encoder.pull() {
            runtime.service(byte);
        }
    }

    fn feed_command(
        runtime: &mut CoreRuntime<
            '_,
            MemStore,
            NoKeyStore,
            NullStateStore,
            MockLink,
            MockLink,
            MockLink,
            512,
        >,
        kind: Kind,
        payload: &[u8],
    ) {
        let mut raw = [0u8; 64];
        let raw_len = Packet::write(NODE, kind, payload, &mut raw).unwrap();
        let mut encoder = Encoder::new(&raw[..raw_len]);
        while let Some(byte) = encoder.pull() {
            runtime.service(byte);
        }
    }

    fn decode_kind(frame: &[u8]) -> Kind {
        decode_packet(frame).1
    }

    fn decode_packet(frame: &[u8]) -> (u8, Kind) {
        let mut scratch = [0u8; 128];
        let mut decoder = Decoder::new();
        let mut done = None;
        for &byte in frame {
            if let Some(n) = decoder.feed(byte, &mut scratch).unwrap() {
                done = Some(n);
            }
        }
        let n = done.expect("response frame did not complete");
        let packet = Packet::parse(&scratch[..n]).unwrap();
        (packet.id, packet.kind)
    }

    #[test]
    fn probe_is_answered_on_the_bus_only() {
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, PersistentState::new(1));
        feed_command(&mut runtime, Kind::BootProbe, &[]);

        assert_eq!(decode_kind(&runtime.bus.written), Kind::BootStatus);
        assert!(
            runtime.prog.written.is_empty(),
            "a probe must not signal the programmer"
        );
    }

    #[test]
    fn first_successful_exchange_marks_app_healthy() {
        let mut state = PersistentState::new(1);
        state.boot_count = 3;
        state.app_health = AppHealth::Unknown;
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, state);

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
    fn committed_image_is_handed_off_once() {
        let ready = PersistentState {
            agent_version: 1,
            app_version: 0,
            staged_version: 9,
            app_health: AppHealth::Unknown,
            staged: StagedState::Ready,
            staged_region: Some(Region::ApplicationCode),
            last_outcome: UpdateOutcome::None,
            program_attempts: 0,
            boot_count: 0,
        };
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, ready);

        // A bare COBS delimiter decodes to no command, so only the
        // pending-program path runs.
        runtime.service(0);
        assert!(
            !runtime.prog.written.is_empty(),
            "a ready image must be signaled to the programmer"
        );
        assert_eq!(decode_kind(&runtime.prog.written), Kind::ProgProgram);
        assert_eq!(
            runtime.dispatcher.agent().status().staged,
            StagedState::Empty
        );

        let sent = runtime.prog.written.len();
        runtime.service(0);
        assert_eq!(runtime.prog.written.len(), sent);
    }

    #[test]
    fn programmer_ok_reply_keeps_success_outcome() {
        let ready = ready_state(Region::CellagentApp);
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, ready);
        runtime
            .prog
            .queue_packet(Kind::ProgResult, &[ProgStatus::Ok.to_code()]);

        // The reply is consumed one byte per try_service call.
        runtime.service(0);
        for _ in 0..32 {
            runtime.try_service();
        }
        assert_eq!(
            runtime.dispatcher.agent().status().last_outcome,
            UpdateOutcome::Success
        );
        assert!(!runtime.awaiting_prog_result);
    }

    #[test]
    fn programmer_failure_reply_records_program_failed() {
        let ready = ready_state(Region::CellagentApp);
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, ready);
        runtime
            .prog
            .queue_packet(Kind::ProgResult, &[ProgStatus::OkReleaseFailed.to_code()]);

        runtime.service(0);
        for _ in 0..32 {
            runtime.try_service();
        }
        assert_eq!(
            runtime.dispatcher.agent().status().last_outcome,
            UpdateOutcome::ProgramFailed
        );
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
    fn agent_bound_frame_is_forwarded_and_reply_relayed() {
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, PersistentState::new(1));
        runtime.agent.queue_packet(Kind::BalancerGateState, &[0x03]);

        feed_from(&mut runtime, AGENT_ID, Kind::ReadBalancerGateState, &[]);

        assert_eq!(
            decode_kind(&runtime.agent.written),
            Kind::ReadBalancerGateState
        );
        assert_eq!(decode_kind(&runtime.bus.written), Kind::BalancerGateState);
    }

    #[test]
    fn silent_agent_gets_a_route_timeout_nack() {
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, PersistentState::new(1));

        feed_from(&mut runtime, AGENT_ID, Kind::SetBalancer, &[0x01]);

        assert_eq!(decode_kind(&runtime.bus.written), Kind::Nack);
    }

    #[test]
    fn own_frames_still_answer_and_do_not_touch_the_agent_link() {
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, PersistentState::new(1));

        feed_command(&mut runtime, Kind::BootProbe, &[]);

        assert_eq!(decode_kind(&runtime.bus.written), Kind::BootStatus);
        assert!(runtime.agent.written.is_empty());
    }

    struct EchoHandler {
        noted: std::vec::Vec<(Kind, u8)>,
        agent_temps: std::vec::Vec<Option<i16>>,
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

        fn note_agent_temp(&mut self, temp: Option<i16>) {
            self.agent_temps.push(temp);
        }
    }

    #[test]
    fn telemetry_kinds_are_answered_by_the_side_handler() {
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1));
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
            agent_temps: std::vec::Vec::new(),
        };
        let runtime = &mut runtime.with_telemetry(&mut handler, NODE);
        feed_command(runtime, Kind::ReadRails, &[]);
        assert_eq!(decode_kind(&runtime.bus.written), Kind::Rails);
    }

    #[test]
    fn boot_kinds_still_reach_the_update_agent_alongside_telemetry() {
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1));
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
            agent_temps: std::vec::Vec::new(),
        };
        let runtime = &mut runtime.with_telemetry(&mut handler, NODE);
        feed_command(runtime, Kind::BootProbe, &[]);
        assert_eq!(decode_kind(&runtime.bus.written), Kind::BootStatus);
    }

    #[test]
    fn forwarded_set_balancer_notifies_the_handler() {
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1));
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
            agent_temps: std::vec::Vec::new(),
        };
        {
            let mut runtime = runtime.with_telemetry(&mut handler, NODE);
            feed_from(&mut runtime, AGENT_ID, Kind::SetBalancer, &[0x03]);
        }
        assert_eq!(handler.noted, std::vec![(Kind::SetBalancer, 0x03)]);
    }

    #[test]
    fn agent_temp_poll_reaches_the_handler_and_not_the_bus() {
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1));
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
            agent_temps: std::vec::Vec::new(),
        };
        {
            let runtime = &mut runtime.with_telemetry(&mut handler, NODE);
            runtime
                .agent
                .queue_packet(Kind::Temperature, &2500i16.to_le_bytes());

            runtime.tick(0);
            runtime.try_poll_agent_temp();
            assert!(runtime.agent.written.is_empty());

            runtime.tick(super::AGENT_TEMP_POLL_TICKS);
            runtime.try_poll_agent_temp();

            let (id, kind) = decode_packet(&runtime.agent.written);
            assert_eq!(kind, Kind::ReadTemperature);
            assert_eq!(id, AGENT_ID);
            assert!(runtime.bus.written.is_empty());
        }
        assert_eq!(handler.agent_temps, std::vec![Some(2500)]);
    }

    #[test]
    fn agent_temp_poll_misses_silently_and_respects_the_cadence() {
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1));
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
            agent_temps: std::vec::Vec::new(),
        };
        {
            let runtime = &mut runtime.with_telemetry(&mut handler, NODE);
            runtime.tick(super::AGENT_TEMP_POLL_TICKS);
            runtime.try_poll_agent_temp();
            let sent = runtime.agent.written.len();

            runtime.try_poll_agent_temp();
            assert_eq!(runtime.agent.written.len(), sent);
            assert!(runtime.bus.written.is_empty());
        }
        assert_eq!(handler.agent_temps, std::vec![None]);
    }

    #[test]
    fn agent_temp_poll_rejects_a_foreign_reply() {
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1));
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
            agent_temps: std::vec::Vec::new(),
        };
        {
            let runtime = &mut runtime.with_telemetry(&mut handler, NODE);
            runtime.agent.queue_packet(Kind::BalancerGateState, &[0x03]);

            runtime.tick(super::AGENT_TEMP_POLL_TICKS);
            runtime.try_poll_agent_temp();
        }
        assert_eq!(handler.agent_temps, std::vec![None]);
    }
}
