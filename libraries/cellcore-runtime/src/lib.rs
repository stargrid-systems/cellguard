//! Drives the `cellcore` update agent over a concrete byte transport.
//!
//! [`CoreRuntime`] ties the hardware-independent [`Dispatcher`] to two links: a
//! field bus (RS485 or debug UART) that carries host commands, and the local
//! link to the `cellprog` programmer. It pumps received bytes through the
//! dispatcher, writes each response back to the bus, and hands a committed
//! image off to the programmer.
//!
//! Both links are any [`embedded_io`] `Read`/`Write`, so the firmware crates
//! pass a HAL `Usart` and stay thin, and this crate is host-testable with a
//! mock link. It knows nothing about pins, clocks, or a specific chip.
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
//! [`UpdateAgent::record_program_failure`]. Application and bootloader
//! handoffs never see a reply, so their outcome stays `Success`, which is
//! accurate: the bootloader owns those flash paths and tracks its own attempts.

#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

use cellboot::image::Region;
use cellboot::io::{ImageStore, KeyStore, StateStore};
use cellcore::update::dispatch::Dispatcher;
use cellcore::update::handoff::{self, PROGRAM_WIRE, RESULT_FRAME};
use cellguard_protocol::{Decoder, Packet, ProgStatus};
use embedded_io::{Read, Write};

/// An in-RAM [`ImageStore`] for bring-up and testing.
///
/// The real board stages images into external SPI EEPROM. This backs the store
/// with a fixed RAM buffer instead, so the receive, verify, and stage path can
/// run on silicon before the board EEPROM wiring is in place. `N` bounds the
/// largest image that can be staged.
pub struct RamImageStore<const N: usize> {
    buf: [u8; N],
}

impl<const N: usize> RamImageStore<N> {
    /// Creates a zeroed store.
    #[must_use]
    pub const fn new() -> Self {
        Self { buf: [0; N] }
    }

    fn range(offset: u32, len: usize) -> Result<(usize, usize), OutOfBounds> {
        let start = usize::try_from(offset).map_err(|_| OutOfBounds)?;
        let end = start.checked_add(len).ok_or(OutOfBounds)?;
        if end > N {
            return Err(OutOfBounds);
        }
        Ok((start, end))
    }
}

impl<const N: usize> Default for RamImageStore<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> ImageStore for RamImageStore<N> {
    type Error = OutOfBounds;

    fn capacity(&self) -> u32 {
        u32::try_from(N).unwrap_or(u32::MAX)
    }

    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OutOfBounds> {
        let (start, end) = Self::range(offset, buf.len())?;
        buf.copy_from_slice(self.buf.get(start..end).ok_or(OutOfBounds)?);
        Ok(())
    }

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), OutOfBounds> {
        let (start, end) = Self::range(offset, data.len())?;
        self.buf
            .get_mut(start..end)
            .ok_or(OutOfBounds)?
            .copy_from_slice(data);
        Ok(())
    }
}

/// The error returned by [`RamImageStore`] when a range leaves the buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutOfBounds;

impl core::fmt::Display for OutOfBounds {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("image store access out of bounds")
    }
}

impl core::error::Error for OutOfBounds {}

/// Re-exported from `cellboot::io`: an [`ImageStore`] that bands two stores
/// into one address space. See [`cellboot::io::BandedStore`].
pub use cellboot::io::{BandedError, BandedStore};

/// Hosts the update agent on a pair of byte links.
///
/// `Bus` is the field bus the host talks on. `Prog` is the local link to the
/// `cellprog` programmer. `RX` sizes the dispatcher receive buffer.
pub struct CoreRuntime<'k, S, K, St, Bus, Prog, const RX: usize> {
    dispatcher: Dispatcher<'k, S, K, St, RX>,
    bus: Bus,
    prog: Prog,
    prog_id: u8,
    /// Whether the running app has confirmed itself healthy this boot. The
    /// runtime calls [`UpdateAgent::confirm_app_healthy`] on the first
    /// successful field-bus exchange, then latches this so it does not
    /// re-persist on every subsequent byte.
    app_confirmed: bool,
    /// Set after a handoff frame is written, cleared when a `ProgResult`
    /// reply arrives (or a garbage frame does). While set, each serviced byte
    /// also polls the programmer link for the reply.
    awaiting_prog_result: bool,
    /// COBS decoder state for the programmer reply.
    prog_decoder: Decoder,
    /// Scratch for one decoded programmer-reply frame.
    prog_scratch: [u8; RESULT_FRAME],
}

impl<'k, S, K, St, Bus, Prog, const RX: usize> CoreRuntime<'k, S, K, St, Bus, Prog, RX>
where
    S: ImageStore,
    K: KeyStore,
    St: StateStore,
    Bus: Read + Write,
    Prog: Read + Write,
{
    /// Creates a runtime around `dispatcher`.
    ///
    /// `bus` carries host commands, `prog` reaches the programmer node
    /// `prog_id`.
    pub const fn new(
        dispatcher: Dispatcher<'k, S, K, St, RX>,
        bus: Bus,
        prog: Prog,
        prog_id: u8,
    ) -> Self {
        Self {
            dispatcher,
            bus,
            prog,
            prog_id,
            app_confirmed: false,
            awaiting_prog_result: false,
            prog_decoder: Decoder::new(),
            prog_scratch: [0; RESULT_FRAME],
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
    /// healthy (resetting the boot counter the bootloader increments), so a
    /// device that answers the field bus cannot be flagged as a crash loop.
    ///
    /// Link errors are swallowed. A dropped bus response is retried on the
    /// next byte because the dispatcher re-derives it from the decoded
    /// command. A dropped handoff is not retried: the staged image is
    /// consumed before signaling (see `hand_off`) to avoid races with the
    /// bootloader self-program path.
    pub fn service(&mut self, byte: u8) {
        if let Some(response) = self.dispatcher.feed(byte) {
            let delivered = self.bus.write_all(response).is_ok();
            // The first delivered response this boot proves the app is alive
            // and answering the field bus, so clear the boot counter the
            // bootloader increments. Latch the flag so we persist at most
            // once per boot rather than on every subsequent byte.
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

    /// Signals the programmer to flash the committed `region`.
    ///
    /// The staged image is consumed and persisted before the frame is written.
    /// If the write fails the update is lost (the outcome is already `Success`
    /// and `pending_program` returns `None`). This ordering prevents the
    /// bootloader from self-programming the same image on the next reboot while
    /// cellprog is simultaneously flashing it over UPDI.
    fn hand_off(&mut self, region: Region) {
        let mut frame = [0u8; PROGRAM_WIRE];
        let Some(len) = handoff::program_frame(self.prog_id, region, &mut frame) else {
            return;
        };
        // Consume (and persist) before signaling, so a reset during programming
        // cannot re-trigger the same handoff on reboot.
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
    use cellboot::io::{ImageStore, NoKeyStore};
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
    ) -> CoreRuntime<'_, MemStore, NoKeyStore, NullStateStore, MockLink, MockLink, 512> {
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
        )
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
    fn banded_store_routes_and_rejects_straddles() {
        use super::{BandedError, BandedStore, RamImageStore};

        let mut store = BandedStore::new(RamImageStore::<64>::new(), RamImageStore::<32>::new());
        assert_eq!(store.capacity(), 96);

        // Low band write is invisible to the high band and vice versa.
        store.write(10, &[0xAA; 4]).unwrap();
        store.write(64, &[0xBB; 4]).unwrap();

        let mut buf = [0u8; 4];
        store.read(10, &mut buf).unwrap();
        assert_eq!(buf, [0xAA; 4]);
        store.read(64, &mut buf).unwrap();
        assert_eq!(buf, [0xBB; 4]);

        // An access crossing the boundary is refused.
        assert_eq!(store.write(62, &[0; 4]), Err(BandedError::OutOfBounds));
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
}
