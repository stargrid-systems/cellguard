//! Drives the `cellcore` update agent over a concrete byte transport.
//!
//! [`CoreRuntime`] ties the hardware-independent [`Dispatcher`] to up to
//! three links: a field bus (RS485 or debug UART) that carries host
//! commands, the local link to the `cellprog` programmer, and an optional
//! link to a downstream node like the `cellagent`. It pumps received bytes
//! through the dispatcher, writes each response back to the bus, and hands a
//! committed image off to the programmer.
//!
//! Both links are any [`embedded_io`] `Read`/`Write`, so the firmware crates
//! pass a HAL `Usart` and stay thin, and this crate is host-testable with a
//! mock link. It knows nothing about pins, clocks, or a specific chip.
//!
//! # Downstream routing
//!
//! Bus frames addressed to `agent_id` are forwarded over the agent link
//! verbatim, and the agent's reply frame is relayed back onto the bus. The
//! runtime keeps its own decoder alongside the dispatcher's: the dispatcher
//! silently ignores foreign-id frames, so the two views never conflict. The
//! forwarded exchange is synchronous - the bus is idle while the agent
//! answers, which the request/response protocol guarantees - and bounded by
//! bounded reads of the agent link. A silent agent produces a
//! `Nack(RouteTimeout)` on the bus so the host exchange always completes.
//!
//! # Handoff
//!
//! When the agent has a committed image ready, the runtime builds the
//! `cellprog` request and sends it over the programmer link. Programming an
//! application or bootloader image resets the core (the programmer halts it
//! over UPDI), so the runtime consumes the staged image first (see
//! [`Dispatcher::take_pending_program`]), which persists the new state, then
//! signals the programmer. A reset mid-programming therefore cannot make the
//! core re-trigger the same flash on reboot.
//!
//! The consume-before-signal ordering means a failed write to the programmer
//! link loses the update silently: the staged image is already cleared and the
//! outcome is `Success`. This is acceptable because the programmer link is a
//! local UART with negligible failure probability, and the alternative
//! (write-first) would race the bootloader self-program path with the cellprog
//! UPDI flash if the programmer resets the core before the runtime can consume.
//!
//! # Programmer reply
//!
//! A cellagent handoff does not reset the core, so the runtime stays alive to
//! read the programmer's `ProgResult` reply. After sending the request it
//! polls the programmer link for one bounded read per serviced byte (the link
//! must therefore have a receive timeout, not block forever). A reported
//! failure flips the persisted outcome to `ProgramFailed` via
//! `UpdateAgent::record_program_failure`. Application and bootloader
//! handoffs never see a reply, so their outcome stays `Success`, which is
//! accurate: the bootloader owns those flash paths and tracks its own attempts.

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
/// small (a mask byte at most), so this only bounds noise.
const AGENT_RX: usize = 64;

/// Bounded reads spent waiting for an agent reply before `RouteTimeout`.
/// One read is one agent-link receive timeout; 40 reads of 2 ms cover any
/// sane reply while keeping a dead node from stalling the bus for long.
const AGENT_REPLY_BUDGET: usize = 40;

/// Scratch for one decoded agent-bound frame, reused to stage the
/// `RouteTimeout` reply payload.
const ROUTE_TIMEOUT_PAYLOAD: [u8; 1] = [NackReason::RouteTimeout.to_code()];

/// Largest telemetry reply payload this runtime can build (the cell snapshot
/// is 1 seq byte plus 4 codes).
const TELEMETRY_PAYLOAD_MAX: usize = 17;

/// What `route` does with a completed frame.
enum RouteAction {
    /// Forward to the agent link (carries the frame length).
    Forward(usize),
    /// Offer to the telemetry handler (carries the request kind and length).
    Telemetry(Kind, usize),
}

/// A side handler for node-local request kinds the update agent does not
/// own, such as the balancing-test telemetry. Implemented by the firmware
/// over the `cellcore` crate's subsystems.
///
/// `handle` receives the request kind and payload plus the runtime's current
/// tick (see [`CoreRuntime::tick`]), writes the reply payload into `out`, and
/// returns the reply kind and payload length. Returning `None` leaves the
/// frame unanswered.
pub trait TelemetryHandler {
    /// Serves one request. See the trait docs.
    ///
    /// # Errors
    ///
    /// None: a handler that cannot serve a request returns `None`, and the
    /// frame stays unanswered.
    fn handle(
        &mut self,
        now: u32,
        kind: Kind,
        payload: &[u8],
        out: &mut [u8],
    ) -> Option<(Kind, usize)>;

    /// Observes a frame routed to the downstream node, after it was written
    /// to the agent link. The default ignores it.
    fn note_forwarded(&mut self, _kind: Kind, _payload: &[u8]) {}

    /// Called once per [`CoreRuntime::tick`], so the handler can run
    /// time-based duties like a heartbeat cadence. The default ignores it.
    fn on_tick(&mut self, _now: u32) {}
}

/// Hosts the update agent on a pair of byte links.
///
/// `Bus` is the field bus the host talks on. `Prog` is the local link to the
/// `cellprog` programmer. `RX` sizes the dispatcher receive buffer.
pub struct CoreRuntime<'k, S, K, St, Bus, Prog, Agent, const RX: usize> {
    dispatcher: Dispatcher<'k, S, K, St, RX>,
    bus: Bus,
    prog: Prog,
    agent: Agent,
    agent_id: u8,
    prog_id: u8,
    node_id: u8,
    /// Whether the app has confirmed itself healthy this boot, so
    /// [`UpdateAgent::confirm_app_healthy`] is persisted at most once per
    /// boot.
    app_confirmed: bool,
    /// Set after a handoff frame is written, cleared when a `ProgResult`
    /// reply arrives (or a garbage frame does). While set, each serviced byte
    /// also polls the programmer link for the reply.
    awaiting_prog_result: bool,
    /// COBS decoder state for the programmer reply.
    prog_decoder: Decoder,
    /// Scratch for one decoded programmer-reply frame.
    prog_scratch: [u8; RESULT_FRAME],
    /// Decoder watching the bus for agent-bound frames.
    route_decoder: Decoder,
    /// Scratch for one decoded agent-bound frame.
    route_scratch: [u8; AGENT_RX],
    /// Decoder for the agent's reply frame.
    reply_decoder: Decoder,
    /// Side handler for node-local telemetry kinds, if installed.
    telemetry: Option<&'k mut dyn TelemetryHandler>,
    /// Current tick, advanced by [`CoreRuntime::tick`].
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
    /// `bus` carries host commands, `prog` reaches the programmer node
    /// `prog_id`, and `agent` reaches the downstream node `agent_id`. Bus
    /// frames addressed to `agent_id` are routed over `agent` (see the
    /// crate's `# Downstream routing` section).
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
            telemetry: None,
            now: 0,
        }
    }

    /// Installs a telemetry handler and this node's bus address. Frames
    /// addressed to `node_id` whose kind the handler owns are answered by it;
    /// the update agent answers its own kinds as before.
    #[must_use]
    pub fn with_telemetry(mut self, handler: &'k mut dyn TelemetryHandler, node_id: u8) -> Self {
        self.telemetry = Some(handler);
        self.node_id = node_id;
        self
    }

    /// Advances the runtime's tick, passed to telemetry handlers to stamp
    /// their refresh windows. Call from the event loop with the real time
    /// base.
    pub fn tick(&mut self, now: u32) {
        self.now = now;
        if let Some(handler) = self.telemetry.as_deref_mut() {
            handler.on_tick(now);
        }
    }

    /// Runs the agent forever.
    ///
    /// Blocks on the bus for one byte at a time and services it. This is the
    /// simple dedicated driver; an application with other duties can call
    /// [`CoreRuntime::try_service`] from its own event loop instead.
    pub fn run(&mut self) -> ! {
        loop {
            self.try_service();
        }
    }

    /// Attempts to read and service one bus byte.
    ///
    /// If no byte arrives within the bus receive timeout, returns immediately
    /// (after polling an in-flight programmer reply). Call this from a custom
    /// event loop that has other periodic duties, like a heartbeat toggle.
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
    /// Feeds the byte to the dispatcher, writes any response back to the bus,
    /// and hands a newly committed image off to the programmer. The first
    /// successful exchange in a boot also marks the running application
    /// healthy, so a device that answers the field bus cannot be flagged as a
    /// crash loop.
    ///
    /// Link errors are swallowed. A dropped bus response is retried on the
    /// next byte because the dispatcher re-derives it from the decoded
    /// command. A dropped handoff is not retried (see the crate's
    /// `# Handoff` section).
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

    /// Watches bus bytes for agent-bound frames and forwards them.
    ///
    /// The dispatcher silently ignores foreign-id frames, so the same byte
    /// can safely feed both decoders. When a complete frame addressed to the
    /// agent arrives, it is re-encoded onto the agent link and the reply is
    /// relayed back onto the bus (see the crate's `# Downstream routing`
    /// section).
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
                // Parse the full packet for the telemetry handler; a frame
                // that does not parse is not ours to answer.
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

    /// Answers a node-local frame through the telemetry handler, if the
    /// handler owns its kind.
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

    /// Writes one decoded frame to the agent link, waits for its reply, and
    /// relays the reply bytes onto the bus. A silent agent gets a
    /// `Nack(RouteTimeout)` so the host exchange always completes.
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

    /// Answers the bus with a `RouteTimeout` nack from this node.
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
    /// Consume-before-signal ordering: see the crate's `# Handoff` section.
    /// A failed write to the link loses the update, because the staged image
    /// was already consumed and the outcome is already `Success`.
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
    /// handoff, one bounded read per call.
    ///
    /// A read timeout or link error leaves the poll armed for the next call.
    /// A completed frame clears it, whatever it contains: a reported failure
    /// is recorded, `Ok` and anything unparsable are not.
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
    /// Concrete test store, pinned to the test capacity.
    type MemStore = MemStoreImpl<CAP>;

    /// A link that records everything written and yields scripted read bytes.
    #[derive(Default)]
    struct MockLink {
        written: std::vec::Vec<u8>,
        readable: std::vec::Vec<u8>,
    }

    impl MockLink {
        /// Encodes a packet as a COBS frame and queues it for reading.
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

    /// Feeds a COBS-encoded frame addressed to `id` byte by byte.
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

    /// Feeds a COBS-encoded command frame byte by byte.
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
        // Boot the app with the bootloader's post-boot counters: boot_count
        // bumped, health still Unknown. The first delivered field-bus
        // response must clear boot_count and flip health to Good.
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
        // Start already staged and ready, as if a commit had just persisted it.
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

        // Any serviced byte drives the handoff. A bare COBS delimiter decodes to
        // no command, so only the pending-program path runs.
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

        // A second byte must not signal again: the image was consumed.
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

        // The reply is consumed one byte per loop tick.
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
        // The agent answers with a BalancerGateState frame.
        runtime.agent.queue_packet(Kind::BalancerGateState, &[0x03]);

        feed_from(&mut runtime, AGENT_ID, Kind::ReadBalancerGateState, &[]);

        // The frame reached the agent link verbatim (decodable, same kind).
        assert_eq!(
            decode_kind(&runtime.agent.written),
            Kind::ReadBalancerGateState
        );
        // The reply was relayed onto the bus.
        assert_eq!(decode_kind(&runtime.bus.written), Kind::BalancerGateState);
    }

    #[test]
    fn silent_agent_gets_a_route_timeout_nack() {
        let mut key = KEY;
        let mut runtime = runtime_with(&mut key, PersistentState::new(1));
        // No reply scripted: the agent stays silent.

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
        let mut key = KEY;
        let runtime = runtime_with(&mut key, PersistentState::new(1));
        let mut handler = EchoHandler {
            noted: std::vec::Vec::new(),
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
        };
        {
            let mut runtime = runtime.with_telemetry(&mut handler, NODE);
            feed_from(&mut runtime, AGENT_ID, Kind::SetBalancer, &[0x03]);
        }
        assert_eq!(handler.noted, std::vec![(Kind::SetBalancer, 0x03)]);
    }
}
