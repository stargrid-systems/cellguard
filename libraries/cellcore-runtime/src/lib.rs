//! Drives the `cellcore` update agent over a concrete byte transport.
//!
//! [`CoreRuntime`] ties the hardware-independent [`Dispatcher`] to two links: a
//! field bus (RS485 or debug UART) that carries host commands, and the local
//! link to the `cellprog` programmer. It pumps received bytes through the
//! dispatcher, writes each response back to the bus, and hands a committed image
//! off to the programmer.
//!
//! Both links are any [`embedded_io`] `Read`/`Write`, so the firmware crates
//! pass a HAL `Usart` and stay thin, and this crate is host-testable with a mock
//! link. It knows nothing about pins, clocks, or a specific chip.
//!
//! # Handoff
//!
//! When the agent has a committed image ready, the runtime builds the
//! `cellprog` request and sends it over the programmer link. Programming resets
//! the core (the programmer halts it over UPDI), so the runtime does not wait
//! for a reply: it consumes the staged image first (see
//! [`Dispatcher::take_pending_program`]), which persists the new state, then
//! signals the programmer. A reset mid-programming therefore cannot make the
//! core re-trigger the same flash on reboot.

#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

use cellboot::image::Region;
use cellboot::io::{ImageStore, KeyStore, StateStore};
use cellcore::update::dispatch::Dispatcher;
use cellcore::update::handoff::{self, PROGRAM_WIRE};
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
}

impl<'k, S, K, St, Bus, Prog, const RX: usize> CoreRuntime<'k, S, K, St, Bus, Prog, RX>
where
    S: ImageStore,
    K: KeyStore,
    St: StateStore,
    Bus: Read + Write,
    Prog: Write,
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
        }
    }

    /// Runs the agent forever.
    ///
    /// Blocks on the bus for one byte at a time and services it. This is the
    /// simple dedicated driver; an application with other duties can call
    /// [`CoreRuntime::service`] from its own event loop instead.
    pub fn run(&mut self) -> ! {
        loop {
            let mut buf = [0u8; 1];
            if self.bus.read_exact(&mut buf).is_ok()
                && let Some(&byte) = buf.first()
            {
                self.service(byte);
            }
        }
    }

    /// Services one received bus byte.
    ///
    /// Feeds the byte to the dispatcher, writes any response back to the bus,
    /// and hands a newly committed image off to the programmer. Link errors are
    /// swallowed: a dropped response or handoff is retried on the next byte, and
    /// the agent state is already durable.
    pub fn service(&mut self, byte: u8) {
        if let Some(response) = self.dispatcher.feed(byte) {
            let _ = self.bus.write_all(response);
        }
        if let Some(region) = self.dispatcher.agent().pending_program() {
            self.hand_off(region);
        }
    }

    /// Signals the programmer to flash the committed `region`.
    fn hand_off(&mut self, region: Region) {
        let mut frame = [0u8; PROGRAM_WIRE];
        let Some(len) = handoff::program_frame(self.prog_id, region, &mut frame) else {
            return;
        };
        // Consume (and persist) before signaling, so a reset during programming
        // cannot re-trigger the same handoff on reboot.
        let _ = self.dispatcher.take_pending_program();
        if let Some(bytes) = frame.get(..len) {
            let _ = self.prog.write_all(bytes);
        }
    }
}

#[cfg(test)]
mod tests {
    use cellboot::image::Region;
    use cellboot::io::{ImageStore, NoKeyStore, StateStore};
    use cellcore::update::dispatch::Dispatcher;
    use cellcore::update::session::{RegionSlot, StagingLayout, UpdateAgent};
    use cellcore::update::state::{AppHealth, PersistentState, StagedState, UpdateOutcome};
    use cellguard_protocol::{Decoder, Encoder, Kind, Packet};

    use super::CoreRuntime;

    const KEY: &[u8] = b"runtime-test-key";
    const TARGET: u16 = 0x33;
    const NODE: u8 = 7;
    const PROG_ID: u8 = 4;
    const CAP: usize = 4096;

    struct MemStore {
        buf: [u8; CAP],
    }

    impl ImageStore for MemStore {
        type Error = ();

        fn capacity(&self) -> u32 {
            u32::try_from(CAP).unwrap()
        }

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()> {
            let start = usize::try_from(offset).map_err(|_| ())?;
            let end = start.checked_add(buf.len()).ok_or(())?;
            buf.copy_from_slice(self.buf.get(start..end).ok_or(())?);
            Ok(())
        }

        fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), ()> {
            let start = usize::try_from(offset).map_err(|_| ())?;
            let end = start.checked_add(data.len()).ok_or(())?;
            self.buf.get_mut(start..end).ok_or(())?.copy_from_slice(data);
            Ok(())
        }
    }

    struct NullStateStore;

    impl StateStore for NullStateStore {
        type Error = ();

        fn load(&mut self, _buf: &mut [u8]) -> Result<(), ()> {
            Err(())
        }

        fn store(&mut self, _data: &[u8]) -> Result<(), ()> {
            Ok(())
        }
    }

    /// A link that records everything written and never yields a read byte.
    #[derive(Default)]
    struct MockLink {
        written: std::vec::Vec<u8>,
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
        fn read(&mut self, _buf: &mut [u8]) -> Result<usize, Self::Error> {
            Ok(0)
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
        }
    }

    fn runtime_with(
        state: PersistentState,
    ) -> CoreRuntime<'static, MemStore, NoKeyStore, NullStateStore, MockLink, MockLink, 512> {
        let agent = UpdateAgent::new(
            MemStore { buf: [0; CAP] },
            layout(),
            TARGET,
            KEY,
            NoKeyStore,
            NullStateStore,
            state,
        );
        let dispatcher = Dispatcher::new(agent, NODE);
        CoreRuntime::new(dispatcher, MockLink::default(), MockLink::default(), PROG_ID)
    }

    /// Feeds a COBS-encoded command frame byte by byte.
    fn feed_command(
        runtime: &mut CoreRuntime<
            'static,
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
        assert_eq!(
            store.write(62, &[0; 4]),
            Err(BandedError::OutOfBounds)
        );
    }

    #[test]
    fn probe_is_answered_on_the_bus_only() {
        let mut runtime = runtime_with(PersistentState::new(1));
        feed_command(&mut runtime, Kind::BootProbe, &[]);

        assert_eq!(decode_kind(&runtime.bus.written), Kind::BootStatus);
        assert!(
            runtime.prog.written.is_empty(),
            "a probe must not signal the programmer"
        );
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
            boot_count: 0,
        };
        let mut runtime = runtime_with(ready);

        // Any serviced byte drives the handoff. A bare COBS delimiter decodes to
        // no command, so only the pending-program path runs.
        runtime.service(0);
        assert!(
            !runtime.prog.written.is_empty(),
            "a ready image must be signaled to the programmer"
        );
        assert_eq!(decode_kind(&runtime.prog.written), Kind::ProgProgram);
        assert_eq!(runtime.dispatcher.agent().status().staged, StagedState::Empty);

        // A second byte must not signal again: the image was consumed.
        let sent = runtime.prog.written.len();
        runtime.service(0);
        assert_eq!(runtime.prog.written.len(), sent);
    }
}
