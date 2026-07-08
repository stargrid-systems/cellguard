//! The device-side update agent state machine.
//!
//! [`UpdateAgent`] runs on the AVR128. It answers [`Command`]s from the host,
//! streams the received payload into an [`ImageStore`] (the external EEPROM),
//! and verifies the image before marking it ready. It never programs flash
//! itself. After a successful commit, [`UpdateAgent::pending_program`] tells the
//! caller which region is ready, so the caller can hand off to the programmer.

use hmac_sha256::HMAC;
use crate::image::{HEADER_LEN, ImageHeader, Region, Verifier};
use crate::io::{ImageStore, KeyStore};
use crate::command::{Command, NackReason, Response};
use crate::state::{PersistentState, StagedState, UpdateOutcome};

const HEADER_LEN_U32: u32 = 64;
const _: () = assert!(HEADER_LEN == HEADER_LEN_U32 as usize);

/// Where one image is staged within an [`ImageStore`].
///
/// The header is written at `offset` and the payload immediately after it, so
/// the region must hold `HEADER_LEN + payload_len` bytes.
#[derive(Debug, Clone, Copy)]
pub struct RegionSlot {
    /// Byte offset of the slot within the store.
    pub offset: u32,
    /// Capacity of the slot in bytes.
    pub capacity: u32,
}

/// Maps image regions to their slots in the staging store.
#[derive(Debug, Clone, Copy)]
pub struct StagingLayout {
    /// Slot for [`Region::ApplicationCode`].
    pub application: RegionSlot,
    /// Slot for [`Region::Bootloader`].
    pub bootloader: RegionSlot,
}

impl StagingLayout {
    const fn slot(&self, region: Region) -> Option<RegionSlot> {
        match region {
            Region::ApplicationCode => Some(self.application),
            Region::Bootloader => Some(self.bootloader),
            // The factory region is not a firmware-update target.
            Region::Factory => None,
        }
    }
}

#[expect(
    clippy::large_enum_variant,
    reason = "the agent holds exactly one session at a time; boxing the large \
              variant would need an allocator, which is not available"
)]
enum Session {
    Idle,
    Receiving(Receiving),
}

struct Receiving {
    header: ImageHeader,
    slot: RegionSlot,
    written: u32,
    verifier: Verifier<HMAC>,
}

/// The device-side update agent.
pub struct UpdateAgent<'k, S, K> {
    store: S,
    layout: StagingLayout,
    target_id: u16,
    key: &'k [u8],
    key_store: K,
    state: PersistentState,
    session: Session,
}

impl<'k, S: ImageStore, K: KeyStore> UpdateAgent<'k, S, K> {
    /// Creates an agent.
    ///
    /// `target_id` is this device's identity, used to reject images built for a
    /// different device. `key` is the shared HMAC key, normally a slice over the
    /// USERROW. `key_store` writes a replacement key (use
    /// [`NoKeyStore`](crate::io::NoKeyStore) in production). `state` is the state
    /// loaded from persistent storage at boot.
    pub const fn new(
        store: S,
        layout: StagingLayout,
        target_id: u16,
        key: &'k [u8],
        key_store: K,
        state: PersistentState,
    ) -> Self {
        Self {
            store,
            layout,
            target_id,
            key,
            key_store,
            state,
            session: Session::Idle,
        }
    }

    /// Returns the current probe-able state.
    #[must_use]
    pub const fn status(&self) -> PersistentState {
        self.state
    }

    /// Returns a shared reference to the staging store.
    #[must_use]
    pub const fn store(&self) -> &S {
        &self.store
    }

    /// Returns the region ready to be programmed after a successful commit.
    ///
    /// The caller uses this to decide when to signal the programmer.
    #[must_use]
    pub const fn pending_program(&self) -> Option<Region> {
        match self.state.staged {
            StagedState::Ready => self.state.staged_region,
            _ => None,
        }
    }

    /// Handles one command and returns the response.
    pub fn handle(&mut self, command: Command<'_>) -> Response {
        match command {
            Command::Probe => Response::Status(self.state),
            Command::Begin { header } => self.on_begin(&header),
            Command::Data { offset, chunk } => self.on_data(offset, chunk),
            Command::Commit => self.on_commit(),
            Command::Abort => self.on_abort(),
            Command::ReplaceKey { new_key, tag } => self.on_replace_key(&new_key, &tag),
        }
    }

    fn on_replace_key(&mut self, new_key: &[u8], tag: &[u8; 32]) -> Response {
        if !crate::mac::authenticate_key_replace(self.key, new_key, tag) {
            return Response::Nack(NackReason::Unauthorized);
        }
        if self.key_store.write_key(new_key).is_err() {
            return Response::Nack(NackReason::StorageError);
        }
        Response::Ack { next_offset: 0 }
    }

    fn on_begin(&mut self, header_bytes: &[u8; HEADER_LEN]) -> Response {
        let Ok((header, verifier)) = Verifier::new(HMAC::new(self.key), header_bytes) else {
            return Response::Nack(NackReason::Malformed);
        };
        if header.target_id != self.target_id {
            return Response::Nack(NackReason::WrongTarget);
        }
        let Some(slot) = self.layout.slot(header.region) else {
            return Response::Nack(NackReason::WrongTarget);
        };
        let needed = HEADER_LEN_U32.saturating_add(header.payload_len);
        if needed > slot.capacity {
            return Response::Nack(NackReason::TooLarge);
        }
        if self.store.write(slot.offset, header_bytes).is_err() {
            self.state.last_outcome = UpdateOutcome::StorageFailed;
            return Response::Nack(NackReason::StorageError);
        }

        self.state.staged = StagedState::Receiving;
        self.state.staged_region = Some(header.region);
        self.state.staged_version = header.fw_version;
        self.session = Session::Receiving(Receiving {
            header,
            slot,
            written: 0,
            verifier,
        });
        Response::Ack { next_offset: 0 }
    }

    fn on_data(&mut self, offset: u32, chunk: &[u8]) -> Response {
        let (slot_offset, payload_len, written) = match &self.session {
            Session::Receiving(rx) => (rx.slot.offset, rx.header.payload_len, rx.written),
            Session::Idle => return Response::Nack(NackReason::BadState),
        };
        if offset != written {
            return Response::Nack(NackReason::OutOfOrder);
        }
        let Ok(len) = u32::try_from(chunk.len()) else {
            return Response::Nack(NackReason::TooLarge);
        };
        let new_written = written.saturating_add(len);
        if new_written > payload_len {
            return Response::Nack(NackReason::TooLarge);
        }
        let addr = slot_offset
            .saturating_add(HEADER_LEN_U32)
            .saturating_add(written);
        if self.store.write(addr, chunk).is_err() {
            self.abort_session(UpdateOutcome::StorageFailed);
            return Response::Nack(NackReason::StorageError);
        }

        if let Session::Receiving(rx) = &mut self.session {
            rx.verifier.feed(chunk);
            rx.written = new_written;
        }
        Response::Ack {
            next_offset: new_written,
        }
    }

    fn on_commit(&mut self) -> Response {
        let Session::Receiving(rx) = core::mem::replace(&mut self.session, Session::Idle) else {
            return Response::Nack(NackReason::BadState);
        };
        if rx.written != rx.header.payload_len {
            self.mark_failed();
            return Response::Nack(NackReason::VerifyFailed);
        }
        if rx.verifier.finish().is_err() {
            self.mark_failed();
            return Response::Nack(NackReason::VerifyFailed);
        }

        self.state.staged = StagedState::Ready;
        self.state.staged_region = Some(rx.header.region);
        self.state.staged_version = rx.header.fw_version;
        self.state.last_outcome = UpdateOutcome::Success;
        Response::Ack {
            next_offset: rx.written,
        }
    }

    const fn on_abort(&mut self) -> Response {
        if matches!(self.session, Session::Receiving(_)) {
            self.abort_session(UpdateOutcome::Aborted);
        }
        Response::Ack { next_offset: 0 }
    }

    const fn abort_session(&mut self, outcome: UpdateOutcome) {
        self.session = Session::Idle;
        self.state.staged = StagedState::Empty;
        self.state.staged_region = None;
        self.state.last_outcome = outcome;
    }

    const fn mark_failed(&mut self) {
        self.session = Session::Idle;
        self.state.staged = StagedState::Empty;
        self.state.staged_region = None;
        self.state.last_outcome = UpdateOutcome::VerifyFailed;
    }
}

#[cfg(test)]
mod tests {
    use super::{RegionSlot, StagingLayout, UpdateAgent};
    use hmac_sha256::HMAC;
    use crate::command::{Command, KEY_LEN, NackReason, Response};
    use crate::image::{HEADER_LEN, ImageHeader, ImageKind, Region};
    use crate::io::{ImageStore, KeyStore, NoKeyStore};
    use crate::mac::{KEY_REPLACE_DOMAIN, authenticate_key_replace};
    use crate::state::{PersistentState, StagedState, UpdateOutcome};

    const KEY: &[u8] = b"session-test-key";
    const TARGET: u16 = 0x2A2A;
    const CAP: usize = 4096;

    struct MemStore {
        buf: [u8; CAP],
    }

    impl MemStore {
        const fn new() -> Self {
            Self { buf: [0u8; CAP] }
        }
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

    fn layout() -> StagingLayout {
        StagingLayout {
            application: RegionSlot { offset: 0, capacity: 2048 },
            bootloader: RegionSlot { offset: 2048, capacity: 2048 },
        }
    }

    fn signed_image(payload: &[u8]) -> [u8; HEADER_LEN] {
        let header = ImageHeader {
            kind: ImageKind::Application,
            region: Region::ApplicationCode,
            target_id: TARGET,
            fw_version: 5,
            payload_len: 0,
            payload_crc32: 0,
            hmac: [0u8; 32],
        };
        let full = header.sign(HMAC::new(KEY), payload).unwrap();
        let mut only_header = [0u8; HEADER_LEN];
        only_header.copy_from_slice(&full);
        only_header
    }

    #[expect(
        clippy::cast_possible_truncation,
        reason = "a wrapping byte ramp is exactly the intended test payload"
    )]
    fn ramp300() -> [u8; 300] {
        core::array::from_fn(|i| i as u8)
    }

    fn run_update(agent: &mut UpdateAgent<MemStore, NoKeyStore>, header: &[u8; HEADER_LEN], payload: &[u8]) -> Response {
        assert!(matches!(
            agent.handle(Command::Begin { header: *header }),
            Response::Ack { next_offset: 0 }
        ));
        for (i, chunk) in payload.chunks(13).enumerate() {
            let offset = u32::try_from(i * 13).unwrap();
            assert!(matches!(
                agent.handle(Command::Data { offset, chunk }),
                Response::Ack { .. }
            ));
        }
        agent.handle(Command::Commit)
    }

    #[test]
    fn happy_path_stages_and_verifies() {
        let payload = ramp300();
        let header = signed_image(&payload);
        let mut agent = UpdateAgent::new(MemStore::new(), layout(), TARGET, KEY, NoKeyStore, PersistentState::new(1));

        assert!(matches!(agent.handle(Command::Probe), Response::Status(_)));
        assert!(matches!(run_update(&mut agent, &header, &payload), Response::Ack { .. }));

        assert_eq!(agent.pending_program(), Some(Region::ApplicationCode));
        assert_eq!(agent.status().staged, StagedState::Ready);
        assert_eq!(agent.status().last_outcome, UpdateOutcome::Success);

        // The store holds the header then the payload.
        let store = agent.store();
        assert_eq!(&store.buf[..HEADER_LEN], &header);
        assert_eq!(&store.buf[HEADER_LEN..HEADER_LEN + payload.len()], payload.as_slice());
    }

    #[test]
    fn tampered_payload_is_rejected() {
        let payload = ramp300();
        let header = signed_image(&payload);
        let mut tampered = payload;
        tampered[100] ^= 0x01;
        let mut agent = UpdateAgent::new(MemStore::new(), layout(), TARGET, KEY, NoKeyStore, PersistentState::new(1));

        assert_eq!(run_update(&mut agent, &header, &tampered), Response::Nack(NackReason::VerifyFailed));
        assert_eq!(agent.pending_program(), None);
        assert_eq!(agent.status().last_outcome, UpdateOutcome::VerifyFailed);
    }

    #[test]
    fn wrong_target_is_rejected() {
        let payload = [1u8, 2, 3];
        let header = signed_image(&payload);
        let mut agent = UpdateAgent::new(MemStore::new(), layout(), 0x9999, KEY, NoKeyStore, PersistentState::new(1));
        assert_eq!(
            agent.handle(Command::Begin { header }),
            Response::Nack(NackReason::WrongTarget)
        );
    }

    #[test]
    fn out_of_order_data_is_rejected() {
        let payload = [1u8, 2, 3, 4, 5, 6];
        let header = signed_image(&payload);
        let mut agent = UpdateAgent::new(MemStore::new(), layout(), TARGET, KEY, NoKeyStore, PersistentState::new(1));
        agent.handle(Command::Begin { header });
        assert_eq!(
            agent.handle(Command::Data { offset: 99, chunk: &payload }),
            Response::Nack(NackReason::OutOfOrder)
        );
    }

    #[test]
    fn data_without_begin_is_rejected() {
        let mut agent = UpdateAgent::new(MemStore::new(), layout(), TARGET, KEY, NoKeyStore, PersistentState::new(1));
        assert_eq!(
            agent.handle(Command::Data { offset: 0, chunk: b"x" }),
            Response::Nack(NackReason::BadState)
        );
    }

    #[test]
    fn abort_clears_session() {
        let payload = [1u8, 2, 3, 4];
        let header = signed_image(&payload);
        let mut agent = UpdateAgent::new(MemStore::new(), layout(), TARGET, KEY, NoKeyStore, PersistentState::new(1));
        agent.handle(Command::Begin { header });
        assert!(matches!(agent.handle(Command::Abort), Response::Ack { .. }));
        assert_eq!(agent.status().last_outcome, UpdateOutcome::Aborted);
        assert_eq!(
            agent.handle(Command::Data { offset: 0, chunk: &payload }),
            Response::Nack(NackReason::BadState)
        );
    }

    /// A key store that accepts writes of a correctly-sized key.
    struct AcceptingKeyStore;

    impl KeyStore for AcceptingKeyStore {
        type Error = ();

        fn write_key(&mut self, key: &[u8]) -> Result<(), ()> {
            if key.len() == KEY_LEN { Ok(()) } else { Err(()) }
        }
    }

    fn replace_key_tag(current_key: &[u8], new_key: &[u8]) -> [u8; 32] {
        let mut mac = HMAC::new(current_key);
        mac.update(KEY_REPLACE_DOMAIN);
        mac.update(new_key);
        mac.finalize()
    }

    #[test]
    fn key_replace_authorized_writes_new_key() {
        let store = AcceptingKeyStore;
        let mut agent = UpdateAgent::new(MemStore::new(), layout(), TARGET, KEY, store, PersistentState::new(1));
        let new_key = [0x5Au8; KEY_LEN];
        let tag = replace_key_tag(KEY, &new_key);

        assert!(matches!(
            agent.handle(Command::ReplaceKey { new_key, tag }),
            Response::Ack { .. }
        ));
        // The self-check on the auth helper mirrors what the device did.
        assert!(authenticate_key_replace(KEY, &new_key, &tag));
    }

    #[test]
    fn key_replace_rejects_bad_tag() {
        let store = AcceptingKeyStore;
        let mut agent = UpdateAgent::new(MemStore::new(), layout(), TARGET, KEY, store, PersistentState::new(1));
        let new_key = [0x5Au8; KEY_LEN];
        let mut tag = replace_key_tag(KEY, &new_key);
        tag[0] ^= 0x01;
        assert_eq!(
            agent.handle(Command::ReplaceKey { new_key, tag }),
            Response::Nack(NackReason::Unauthorized)
        );
    }

    #[test]
    fn key_replace_rejected_when_locked() {
        let mut agent = UpdateAgent::new(MemStore::new(), layout(), TARGET, KEY, NoKeyStore, PersistentState::new(1));
        let new_key = [0x5Au8; KEY_LEN];
        let tag = replace_key_tag(KEY, &new_key);
        // Authentication passes, but a locked key store refuses the write.
        assert_eq!(
            agent.handle(Command::ReplaceKey { new_key, tag }),
            Response::Nack(NackReason::StorageError)
        );
    }
}
