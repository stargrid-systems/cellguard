//! The device-side update agent state machine.
//!
//! [`UpdateAgent`] answers [`Command`]s from the host, streams the payload
//! into an [`ImageStore`], and verifies the image before marking it ready.
//! It never programs flash itself.

use cellboot::image::{HEADER_LEN, HEADER_LEN_U32, ImageHeader, ImageKind, Region};
use cellboot::io::{ImageStore, KeyStore, StateStore};
use cellboot::state::{AppHealth, PersistentState, StagedState, UpdateOutcome};
use cellguard_panic::PanicRecord;
use hmac_sha256::HMAC;

use crate::update::command::{Command, KEY_LEN, NackReason, Response};
use crate::update::verify::Verifier;

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
    /// Slot for [`Region::CellagentApp`].
    pub cellagent: RegionSlot,
}

impl StagingLayout {
    const fn slot(&self, region: Region) -> Option<RegionSlot> {
        match region {
            Region::ApplicationCode => Some(self.application),
            Region::Bootloader => Some(self.bootloader),
            Region::CellagentApp => Some(self.cellagent),
            // The factory region, and any region added later, is not a
            // firmware-update target.
            _ => None,
        }
    }
}

#[expect(
    clippy::large_enum_variant,
    reason = "the agent holds exactly one session at a time; boxing the large variant would need \
              an allocator, which is not available"
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
pub struct UpdateAgent<'k, S, K, St> {
    store: S,
    layout: StagingLayout,
    target_id: u16,
    cellagent_target_id: u16,
    key: &'k mut [u8],
    key_store: K,
    state_store: St,
    state: PersistentState,
    session: Session,
    /// Cached last panic record, reported on `PanicProbe`.
    panic_record: Option<PanicRecord>,
}

impl<'k, S: ImageStore, K: KeyStore, St: StateStore> UpdateAgent<'k, S, K, St> {
    /// Creates an agent.
    ///
    /// `target_id` is this device's identity and `cellagent_target_id` is the
    /// cellagent's, used to verify cellagent images relayed through the
    /// cellcore. `key` is the shared HMAC key buffer, updated in place on a
    /// successful key replacement so the new key takes effect immediately.
    /// Use [`NoKeyStore`](cellboot::io::NoKeyStore) in production to disable
    /// key replacement. `state` is the state loaded at boot (see
    /// [`cellboot::state::load`]).
    #[expect(
        clippy::too_many_arguments,
        reason = "each argument is a distinct, required piece of hardware state"
    )]
    pub const fn new(
        store: S,
        layout: StagingLayout,
        target_id: u16,
        cellagent_target_id: u16,
        key: &'k mut [u8; KEY_LEN],
        key_store: K,
        state_store: St,
        state: PersistentState,
    ) -> Self {
        Self {
            store,
            layout,
            target_id,
            cellagent_target_id,
            key,
            key_store,
            state_store,
            state,
            session: Session::Idle,
            panic_record: None,
        }
    }

    /// Caches the last panic record so a later `PanicProbe` reports it. Call
    /// this once at boot after reading the slot from EEPROM.
    pub const fn set_panic_record(&mut self, record: Option<PanicRecord>) {
        self.panic_record = record;
    }

    /// Returns the current probe-able state.
    #[must_use]
    pub const fn status(&self) -> PersistentState {
        self.state
    }

    /// Marks the running application as healthy and clears the boot counter.
    ///
    /// The bootloader counts boots while the app stays unconfirmed and flips
    /// `app_health` to [`Bad`](cellboot::state::AppHealth::Bad) once the count
    /// reaches
    /// [`BOOT_HEALTH_THRESHOLD`](cellboot::state::BOOT_HEALTH_THRESHOLD).
    /// Call this once the app has proven itself alive.
    pub fn confirm_app_healthy(&mut self) {
        self.state.app_health = AppHealth::Good;
        self.state.boot_count = 0;
        let _ = self.state_store.store(&self.state.serialize());
    }

    /// Records that the programmer failed to flash a handed-off image.
    ///
    /// The handoff already recorded `Success` before the programmer ran, so
    /// this flips the persisted outcome to `ProgramFailed` and keeps a probe
    /// honest.
    pub fn record_program_failure(&mut self) {
        if self.state.last_outcome != UpdateOutcome::ProgramFailed {
            self.state.last_outcome = UpdateOutcome::ProgramFailed;
            let _ = self.state_store.store(&self.state.serialize());
        }
    }

    /// Returns the region ready to be programmed after a successful commit.
    #[must_use]
    pub const fn pending_program(&self) -> Option<Region> {
        match self.state.staged {
            StagedState::Ready => self.state.staged_region,
            _ => None,
        }
    }

    /// Consumes the staged image as it is handed off to the programmer.
    ///
    /// Returns the region to program, or `None` when nothing is staged and
    /// ready. Programming resets the core (the programmer halts it over
    /// UPDI), so the core never sees the result and the handoff is final:
    /// the staged image is cleared, an application image advances the
    /// recorded `app_version`, and the outcome is recorded as a success.
    /// A state-store write failure is not surfaced, since the in-RAM state
    /// still drives the current boot.
    ///
    /// There is no rollback enforcement: if programming never happens, the
    /// old image keeps running. Dropping an update is safe.
    #[must_use]
    pub fn take_pending_program(&mut self) -> Option<Region> {
        let region = self.pending_program()?;
        self.state.mark_programmed(region);
        let _ = self.state_store.store(&self.state.serialize());
        Some(region)
    }

    /// Handles one command and returns the response.
    ///
    /// A changed state is written through to the [`StateStore`] so a probe
    /// after a reset reflects reality. `Data` does not change the state, so
    /// a large transfer causes no store writes. The write-through is
    /// best-effort: a store failure is not reported here.
    pub fn handle(&mut self, command: Command<'_>) -> Response {
        let before = self.state;
        let response = match command {
            Command::Probe => Response::Status(self.state),
            Command::PanicProbe => Response::PanicStatus(self.panic_record),
            Command::Begin { header } => self.on_begin(&header),
            Command::Data { offset, chunk } => self.on_data(offset, chunk),
            Command::Commit => self.on_commit(),
            Command::Abort => self.on_abort(),
            Command::ReplaceKey { new_key, tag } => self.on_replace_key(&new_key, &tag),
        };
        if self.state != before {
            let _ = self.state_store.store(&self.state.serialize());
        }
        response
    }

    fn on_replace_key(&mut self, new_key: &[u8], tag: &[u8; 32]) -> Response {
        if !crate::update::mac::authenticate_key_replace(self.key, new_key, tag) {
            return Response::Nack(NackReason::Unauthorized);
        }
        if self.key_store.write_key(new_key).is_err() {
            return Response::Nack(NackReason::StorageError);
        }
        self.key.copy_from_slice(new_key);
        Response::Ack { next_offset: 0 }
    }

    fn on_begin(&mut self, header_bytes: &[u8; HEADER_LEN]) -> Response {
        let Ok((header, verifier)) = Verifier::new(HMAC::new(&*self.key), header_bytes) else {
            return Response::Nack(NackReason::Malformed);
        };
        let expected_id = match header.region {
            Region::CellagentApp => self.cellagent_target_id,
            _ => self.target_id,
        };
        if header.target_id != expected_id {
            return Response::Nack(NackReason::WrongTarget);
        }
        let Some(slot) = self.layout.slot(header.region) else {
            return Response::Nack(NackReason::WrongTarget);
        };
        // The image kind must match the region: an image signed for one kind
        // must never land in the other kind's slot.
        let kind_matches = match header.kind {
            ImageKind::Bootloader => header.region == Region::Bootloader,
            ImageKind::Application => matches!(
                header.region,
                Region::ApplicationCode | Region::CellagentApp
            ),
            _ => false,
        };
        if !kind_matches {
            return Response::Nack(NackReason::WrongTarget);
        }
        let needed = HEADER_LEN_U32.saturating_add(header.payload_len);
        if needed > slot.capacity {
            return Response::Nack(NackReason::TooLarge);
        }
        if self.store.write(slot.offset, header_bytes).is_err() {
            self.abort_session(UpdateOutcome::StorageFailed);
            return Response::Nack(NackReason::StorageError);
        }

        // A new transfer discards a committed image that was never
        // programmed. Record it as host-aborted so the outcome is honest.
        if self.state.staged == StagedState::Ready {
            self.state.last_outcome = UpdateOutcome::Aborted;
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
            self.abort_session(UpdateOutcome::VerifyFailed);
            return Response::Nack(NackReason::VerifyFailed);
        }
        if rx.verifier.finish().is_err() {
            self.abort_session(UpdateOutcome::VerifyFailed);
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
}

#[cfg(test)]
mod tests {
    use core::cell::RefCell;

    use cellboot::image::{HEADER_LEN, ImageHeader, ImageKind, Region};
    use cellboot::io::{ImageStore, KeyStore, NoKeyStore, StateStore};
    use cellboot::state::{PersistentState, STATE_LEN, StagedState, UpdateOutcome};
    use cellboot::testutil::{
        MemStore as MemStoreImpl, NullStateStore, SharedImageStore, SharedStore,
    };
    use hmac_sha256::HMAC;

    use super::{RegionSlot, StagingLayout, UpdateAgent};
    use crate::update::command::{Command, KEY_LEN, NackReason, Response};
    use crate::update::mac::{KEY_REPLACE_DOMAIN, authenticate_key_replace};

    const KEY: [u8; 16] = *b"session-test-key";
    const TARGET: u16 = 0x2A2A;
    const CELLAGENT_TARGET: u16 = 0x2B2B;
    const CAP: usize = 4096;
    /// Concrete test store, pinned to the test capacity.
    type MemStore = MemStoreImpl<CAP>;

    fn layout() -> StagingLayout {
        StagingLayout {
            application: RegionSlot {
                offset: 0,
                capacity: 2048,
            },
            bootloader: RegionSlot {
                offset: 2048,
                capacity: 1024,
            },
            cellagent: RegionSlot {
                offset: 3072,
                capacity: 1024,
            },
        }
    }

    fn signed_image_with(payload: &[u8], key: &[u8]) -> [u8; HEADER_LEN] {
        let header = ImageHeader {
            kind: ImageKind::Application,
            region: Region::ApplicationCode,
            target_id: TARGET,
            fw_version: 5,
            payload_len: 0,
            payload_crc32: 0,
            hmac: [0u8; 32],
        };
        let full = crate::update::verify::sign(header, HMAC::new(key), payload).unwrap();
        let mut only_header = [0u8; HEADER_LEN];
        only_header.copy_from_slice(&full);
        only_header
    }

    fn signed_image(payload: &[u8]) -> [u8; HEADER_LEN] {
        signed_image_with(payload, &KEY)
    }

    #[expect(
        clippy::cast_possible_truncation,
        reason = "a wrapping byte ramp is exactly the intended test payload"
    )]
    fn ramp300() -> [u8; 300] {
        core::array::from_fn(|i| i as u8)
    }

    fn run_update<S: ImageStore, St: StateStore>(
        agent: &mut UpdateAgent<'_, S, NoKeyStore, St>,
        header: &[u8; HEADER_LEN],
        payload: &[u8],
    ) -> Response {
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
        let mut key = KEY;
        let backing = RefCell::new([0u8; CAP]);
        let mut agent = UpdateAgent::new(
            SharedImageStore::new(&backing),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );

        assert!(matches!(agent.handle(Command::Probe), Response::Status(_)));
        assert!(matches!(
            run_update(&mut agent, &header, &payload),
            Response::Ack { .. }
        ));

        assert_eq!(agent.pending_program(), Some(Region::ApplicationCode));
        assert_eq!(agent.status().staged, StagedState::Ready);
        assert_eq!(agent.status().last_outcome, UpdateOutcome::Success);

        let mut staged = [0u8; HEADER_LEN + 300];
        SharedImageStore::new(&backing)
            .read(0, &mut staged)
            .unwrap();
        assert_eq!(&staged[..HEADER_LEN], &header);
        assert_eq!(&staged[HEADER_LEN..HEADER_LEN + payload.len()], &payload);
    }

    #[test]
    fn tampered_payload_is_rejected() {
        let payload = ramp300();
        let header = signed_image(&payload);
        let mut tampered = payload;
        tampered[100] ^= 0x01;
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );

        assert_eq!(
            run_update(&mut agent, &header, &tampered),
            Response::Nack(NackReason::VerifyFailed)
        );
        assert_eq!(agent.pending_program(), None);
        assert_eq!(agent.status().last_outcome, UpdateOutcome::VerifyFailed);
    }

    #[test]
    fn wrong_target_is_rejected() {
        let payload = [1u8, 2, 3];
        let header = signed_image(&payload);
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            0x9999,
            CELLAGENT_TARGET,
            &mut key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );
        assert_eq!(
            agent.handle(Command::Begin { header }),
            Response::Nack(NackReason::WrongTarget)
        );
    }

    #[test]
    fn kind_region_mismatch_is_rejected() {
        let payload = [1u8, 2, 3];
        let header = {
            let image = ImageHeader {
                kind: ImageKind::Bootloader,
                region: Region::ApplicationCode,
                target_id: TARGET,
                fw_version: 5,
                payload_len: 0,
                payload_crc32: 0,
                hmac: [0u8; 32],
            };
            crate::update::verify::sign(image, HMAC::new(KEY), &payload).unwrap()
        };
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );
        assert_eq!(
            agent.handle(Command::Begin { header }),
            Response::Nack(NackReason::WrongTarget)
        );
    }

    #[test]
    fn new_begin_over_a_ready_image_records_aborted() {
        let payload = ramp300();
        let header = signed_image(&payload);
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );
        assert!(matches!(
            run_update(&mut agent, &header, &payload),
            Response::Ack { .. }
        ));
        assert_eq!(agent.status().last_outcome, UpdateOutcome::Success);

        assert!(matches!(
            agent.handle(Command::Begin { header }),
            Response::Ack { next_offset: 0 }
        ));
        assert_eq!(agent.status().last_outcome, UpdateOutcome::Aborted);
    }

    #[test]
    fn out_of_order_data_is_rejected() {
        let payload = [1u8, 2, 3, 4, 5, 6];
        let header = signed_image(&payload);
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );
        agent.handle(Command::Begin { header });
        assert_eq!(
            agent.handle(Command::Data {
                offset: 99,
                chunk: &payload
            }),
            Response::Nack(NackReason::OutOfOrder)
        );
    }

    #[test]
    fn data_without_begin_is_rejected() {
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );
        assert_eq!(
            agent.handle(Command::Data {
                offset: 0,
                chunk: b"x"
            }),
            Response::Nack(NackReason::BadState)
        );
    }

    #[test]
    fn abort_clears_session() {
        let payload = [1u8, 2, 3, 4];
        let header = signed_image(&payload);
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );
        agent.handle(Command::Begin { header });
        assert!(matches!(agent.handle(Command::Abort), Response::Ack { .. }));
        assert_eq!(agent.status().last_outcome, UpdateOutcome::Aborted);
        assert_eq!(
            agent.handle(Command::Data {
                offset: 0,
                chunk: &payload
            }),
            Response::Nack(NackReason::BadState)
        );
    }

    /// A key store that accepts writes of a correctly-sized key.
    struct AcceptingKeyStore;

    impl KeyStore for AcceptingKeyStore {
        type Error = ();

        fn write_key(&mut self, key: &[u8]) -> Result<(), ()> {
            if key.len() == KEY_LEN {
                Ok(())
            } else {
                Err(())
            }
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
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            store,
            NullStateStore,
            PersistentState::new(1),
        );
        let new_key = [0x5Au8; KEY_LEN];
        let tag = replace_key_tag(&KEY, &new_key);

        assert!(matches!(
            agent.handle(Command::ReplaceKey { new_key, tag }),
            Response::Ack { .. }
        ));
        assert!(authenticate_key_replace(&KEY, &new_key, &tag));
    }

    #[test]
    fn key_replace_takes_effect_immediately() {
        let store = AcceptingKeyStore;
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            store,
            NullStateStore,
            PersistentState::new(1),
        );
        let new_key = [0x5Au8; KEY_LEN];
        let tag = replace_key_tag(&KEY, &new_key);

        assert!(matches!(
            agent.handle(Command::ReplaceKey { new_key, tag }),
            Response::Ack { .. }
        ));

        let payload = ramp300();
        let header = signed_image_with(&payload, &new_key);
        assert!(matches!(
            agent.handle(Command::Begin { header }),
            Response::Ack { .. }
        ));
    }

    #[test]
    fn key_replace_rejects_bad_tag() {
        let store = AcceptingKeyStore;
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            store,
            NullStateStore,
            PersistentState::new(1),
        );
        let new_key = [0x5Au8; KEY_LEN];
        let mut tag = replace_key_tag(&KEY, &new_key);
        tag[0] ^= 0x01;
        assert_eq!(
            agent.handle(Command::ReplaceKey { new_key, tag }),
            Response::Nack(NackReason::Unauthorized)
        );
    }

    #[test]
    fn key_replace_rejected_when_locked() {
        let mut key = KEY;
        let mut agent = UpdateAgent::new(
            MemStore::new(),
            layout(),
            TARGET,
            CELLAGENT_TARGET,
            &mut key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );
        let new_key = [0x5Au8; KEY_LEN];
        let tag = replace_key_tag(&KEY, &new_key);
        assert_eq!(
            agent.handle(Command::ReplaceKey { new_key, tag }),
            Response::Nack(NackReason::StorageError)
        );
    }

    #[test]
    fn state_persists_across_reset() {
        let backing: RefCell<Option<[u8; STATE_LEN]>> = RefCell::new(None);
        let payload = ramp300();
        let header = signed_image(&payload);

        {
            let mut key = KEY;
            let mut agent = UpdateAgent::new(
                MemStore::new(),
                layout(),
                TARGET,
                CELLAGENT_TARGET,
                &mut key,
                NoKeyStore,
                SharedStore::new(&backing),
                PersistentState::new(1),
            );
            assert!(matches!(
                run_update(&mut agent, &header, &payload),
                Response::Ack { .. }
            ));
        }

        let restored = cellboot::state::load(&mut SharedStore::new(&backing), 1);
        assert_eq!(restored.staged, StagedState::Ready);
        assert_eq!(restored.staged_region, Some(Region::ApplicationCode));
        assert_eq!(restored.last_outcome, UpdateOutcome::Success);
        assert_eq!(restored.staged_version, 5);
    }

    #[test]
    fn abort_is_persisted() {
        let backing: RefCell<Option<[u8; STATE_LEN]>> = RefCell::new(None);
        let payload = [1u8, 2, 3, 4];
        let header = signed_image(&payload);
        {
            let mut key = KEY;
            let mut agent = UpdateAgent::new(
                MemStore::new(),
                layout(),
                TARGET,
                CELLAGENT_TARGET,
                &mut key,
                NoKeyStore,
                SharedStore::new(&backing),
                PersistentState::new(1),
            );
            agent.handle(Command::Begin { header });
            agent.handle(Command::Abort);
        }
        let restored = cellboot::state::load(&mut SharedStore::new(&backing), 1);
        assert_eq!(restored.staged, StagedState::Empty);
        assert_eq!(restored.last_outcome, UpdateOutcome::Aborted);
    }

    /// An image store whose writes always fail.
    struct FailingStore;

    impl ImageStore for FailingStore {
        type Error = ();

        fn capacity(&self) -> u32 {
            4096
        }

        fn read(&mut self, _offset: u32, _buf: &mut [u8]) -> Result<(), ()> {
            Err(())
        }

        fn write(&mut self, _offset: u32, _data: &[u8]) -> Result<(), ()> {
            Err(())
        }
    }

    /// A `Begin` whose store write fails must clear a previously staged
    /// image: a stale `Ready` must not survive to trigger an unintended
    /// handoff.
    #[test]
    fn begin_storage_failure_clears_stale_ready() {
        let backing: RefCell<Option<[u8; STATE_LEN]>> = RefCell::new(None);
        let payload = ramp300();
        let header = signed_image(&payload);

        {
            let mut key = KEY;
            let mut agent = UpdateAgent::new(
                MemStore::new(),
                layout(),
                TARGET,
                CELLAGENT_TARGET,
                &mut key,
                NoKeyStore,
                SharedStore::new(&backing),
                PersistentState::new(1),
            );
            assert!(matches!(
                run_update(&mut agent, &header, &payload),
                Response::Ack { .. }
            ));
            assert_eq!(agent.pending_program(), Some(Region::ApplicationCode));
        }

        {
            let mut key = KEY;
            let state = cellboot::state::load(&mut SharedStore::new(&backing), 1);
            let mut agent = UpdateAgent::new(
                FailingStore,
                layout(),
                TARGET,
                CELLAGENT_TARGET,
                &mut key,
                NoKeyStore,
                SharedStore::new(&backing),
                state,
            );
            assert_eq!(
                agent.handle(Command::Begin { header }),
                Response::Nack(NackReason::StorageError)
            );
            assert_eq!(agent.pending_program(), None);
            assert_eq!(agent.status().staged, StagedState::Empty);
            assert_eq!(agent.status().last_outcome, UpdateOutcome::StorageFailed);
        }

        let restored = cellboot::state::load(&mut SharedStore::new(&backing), 1);
        assert_eq!(restored.staged, StagedState::Empty);
        assert_eq!(restored.last_outcome, UpdateOutcome::StorageFailed);
    }
}
