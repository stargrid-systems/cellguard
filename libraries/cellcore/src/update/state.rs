//! The persistent, probe-able updater state.
//!
//! [`PersistentState`] is the small record the updater keeps in a
//! [`cellboot::io::StateStore`]. It survives a program-memory rewrite and is
//! what a `Probe` command reports back, so an operator can ask a device what
//! firmware it runs, whether that firmware is healthy, and how the last update
//! went.

use core::fmt;

use cellboot::io::StateStore;

/// Loads the persisted state, falling back to a fresh one on any problem.
///
/// A read error, a wrong length, a bad CRC, or an unknown field all resolve to
/// [`PersistentState::new`], so a blank or corrupt store never blocks boot.
/// Call this once at boot and pass the result to
/// [`UpdateAgent::new`](crate::update::session::UpdateAgent::new).
pub fn load<St: StateStore>(store: &mut St, agent_version: u32) -> PersistentState {
    let mut buf = [0u8; STATE_LEN];
    match store.load(&mut buf) {
        Ok(()) => {
            PersistentState::parse(&buf).unwrap_or_else(|_| PersistentState::new(agent_version))
        }
        Err(_) => PersistentState::new(agent_version),
    }
}

/// Serialized length of a [`PersistentState`] record in bytes.
pub const STATE_LEN: usize = 28;

/// State record format version understood by this crate.
pub const STATE_FORMAT_VERSION: u8 = 1;

/// Default ceiling on `boot_count` before the bootloader declares the
/// application unhealthy.
///
/// The bootloader increments [`PersistentState::boot_count`] on every boot
/// that hands control to the app. If the counter reaches this value without
/// the app calling
/// [`UpdateAgent::confirm_app_healthy`][crate::update::session::UpdateAgent::confirm_app_healthy],
/// the bootloader sets [`AppHealth::Bad`]. A device that reboots in a loop
/// (the app panics before it can confirm) is flagged within this many boots.
pub const BOOT_HEALTH_THRESHOLD: u8 = 5;

/// Sentinel region code meaning "no staged image".
const NO_REGION: u8 = 0xFF;

/// Health of the currently installed application.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum AppHealth {
    /// Health has not been established yet this session.
    #[default]
    Unknown,
    /// The application checked out and confirmed itself.
    Good,
    /// The application failed its integrity check or never confirmed.
    Bad,
}

impl AppHealth {
    const fn to_code(self) -> u8 {
        match self {
            Self::Unknown => 0,
            Self::Good => 1,
            Self::Bad => 2,
        }
    }

    const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Unknown),
            1 => Some(Self::Good),
            2 => Some(Self::Bad),
            _ => None,
        }
    }
}

/// State of the image staged in external storage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum StagedState {
    /// No image is staged.
    #[default]
    Empty,
    /// An image is being received and is not yet complete.
    Receiving,
    /// A complete, verified image is staged and ready to program.
    Ready,
}

impl StagedState {
    const fn to_code(self) -> u8 {
        match self {
            Self::Empty => 0,
            Self::Receiving => 1,
            Self::Ready => 2,
        }
    }

    const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::Empty),
            1 => Some(Self::Receiving),
            2 => Some(Self::Ready),
            _ => None,
        }
    }
}

/// Result of the most recent update attempt.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[non_exhaustive]
pub enum UpdateOutcome {
    /// No update has been attempted since the state was created.
    #[default]
    None,
    /// The most recent update was received and verified successfully.
    Success,
    /// The most recent update failed verification.
    VerifyFailed,
    /// The most recent update failed while writing to storage.
    StorageFailed,
    /// The most recent update was aborted by the host.
    Aborted,
    /// Programming the staged image into flash failed.
    ProgramFailed,
}

impl UpdateOutcome {
    const fn to_code(self) -> u8 {
        match self {
            Self::None => 0,
            Self::Success => 1,
            Self::VerifyFailed => 2,
            Self::StorageFailed => 3,
            Self::Aborted => 4,
            Self::ProgramFailed => 5,
        }
    }

    const fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(Self::None),
            1 => Some(Self::Success),
            2 => Some(Self::VerifyFailed),
            3 => Some(Self::StorageFailed),
            4 => Some(Self::Aborted),
            5 => Some(Self::ProgramFailed),
            _ => None,
        }
    }
}

/// The probe-able updater state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PersistentState {
    /// Version of the update agent and programmer firmware. Informational.
    pub agent_version: u32,
    /// Version of the installed application. Informational.
    pub app_version: u32,
    /// Version of the staged image, meaningful when `staged` is not `Empty`.
    pub staged_version: u32,
    /// Health of the installed application.
    pub app_health: AppHealth,
    /// State of the staged image.
    pub staged: StagedState,
    /// Region the staged image targets, or `None` when nothing is staged.
    pub staged_region: Option<cellboot::image::Region>,
    /// Result of the most recent update attempt.
    pub last_outcome: UpdateOutcome,
    /// Bootloader self-program attempts for the current staged image.
    /// Incremented on each failed attempt; cleared on success or when the
    /// bootloader gives up. Meaningful only to the bootloader.
    pub program_attempts: u8,
    /// Boots since the application last confirmed itself.
    pub boot_count: u16,
}

impl PersistentState {
    /// Creates a fresh state for an agent at `agent_version`.
    #[must_use]
    pub const fn new(agent_version: u32) -> Self {
        Self {
            agent_version,
            app_version: 0,
            staged_version: 0,
            app_health: AppHealth::Unknown,
            staged: StagedState::Empty,
            staged_region: None,
            last_outcome: UpdateOutcome::None,
            program_attempts: 0,
            boot_count: 0,
        }
    }

    /// Marks the staged image as successfully programmed and hands it off.
    ///
    /// Clears the staged slot, records `Success`, and resets the
    /// `program_attempts` counter. For an application image it also advances
    /// `app_version` to the staged version and resets `app_health` and
    /// `boot_count`, so the new app starts from a clean health slate.
    ///
    /// This is the single transition used by both
    /// [`UpdateAgent::take_pending_program`](crate::update::session::UpdateAgent::take_pending_program)
    /// and the bootloader's self-program path, so the two cannot drift.
    pub fn mark_programmed(&mut self, region: cellboot::image::Region) {
        if region == cellboot::image::Region::ApplicationCode {
            self.app_version = self.staged_version;
            self.app_health = AppHealth::Unknown;
            self.boot_count = 0;
        }
        self.staged = StagedState::Empty;
        self.staged_region = None;
        self.last_outcome = UpdateOutcome::Success;
        self.program_attempts = 0;
    }
    /// Marks the staged image as permanently failed: attempts exhausted or the
    /// error is not recoverable by retrying.
    ///
    /// Clears the staged slot, records `ProgramFailed`, and resets
    /// `program_attempts`. The installed app (if any) keeps running.
    pub const fn mark_program_failed(&mut self) {
        self.staged = StagedState::Empty;
        self.staged_region = None;
        self.last_outcome = UpdateOutcome::ProgramFailed;
        self.program_attempts = 0;
    }

    /// Serializes the state into its canonical, CRC-protected byte form.
    #[must_use]
    pub fn serialize(&self) -> [u8; STATE_LEN] {
        let mut out = [0u8; STATE_LEN];
        out[0] = STATE_FORMAT_VERSION;
        out[1] = self.app_health.to_code();
        out[2] = self.staged.to_code();
        out[3] = self.last_outcome.to_code();
        out[4] = self
            .staged_region
            .map_or(NO_REGION, cellboot::image::Region::to_code);
        out[5] = self.program_attempts;
        out[6..8].copy_from_slice(&self.boot_count.to_le_bytes());
        out[8..12].copy_from_slice(&self.agent_version.to_le_bytes());
        out[12..16].copy_from_slice(&self.app_version.to_le_bytes());
        out[16..20].copy_from_slice(&self.staged_version.to_le_bytes());
        let crc = crc::checksum32(&out[0..24]);
        out[24..28].copy_from_slice(&crc.to_le_bytes());
        out
    }

    /// Parses a state record from its bytes.
    ///
    /// # Errors
    ///
    /// Returns [`StateError`] if the record is the wrong length, has a bad CRC,
    /// an unknown format version, or an unknown field value. A caller that gets
    /// an error should fall back to [`PersistentState::new`].
    pub fn parse(bytes: &[u8; STATE_LEN]) -> Result<Self, StateError> {
        let stored_crc = u32::from_le_bytes([bytes[24], bytes[25], bytes[26], bytes[27]]);
        if crc::checksum32(&bytes[0..24]) != stored_crc {
            return Err(StateError::BadCrc);
        }
        if bytes[0] != STATE_FORMAT_VERSION {
            return Err(StateError::UnsupportedFormat(bytes[0]));
        }
        let app_health = AppHealth::from_code(bytes[1]).ok_or(StateError::BadField)?;
        let staged = StagedState::from_code(bytes[2]).ok_or(StateError::BadField)?;
        let last_outcome = UpdateOutcome::from_code(bytes[3]).ok_or(StateError::BadField)?;
        let staged_region = if bytes[4] == NO_REGION {
            None
        } else {
            Some(cellboot::image::Region::from_code(bytes[4]).ok_or(StateError::BadField)?)
        };

        let program_attempts = bytes[5];
        let boot_count = u16::from_le_bytes([bytes[6], bytes[7]]);
        let agent_version = u32::from_le_bytes([bytes[8], bytes[9], bytes[10], bytes[11]]);
        let app_version = u32::from_le_bytes([bytes[12], bytes[13], bytes[14], bytes[15]]);
        let staged_version = u32::from_le_bytes([bytes[16], bytes[17], bytes[18], bytes[19]]);

        Ok(Self {
            agent_version,
            app_version,
            staged_version,
            app_health,
            staged,
            staged_region,
            last_outcome,
            program_attempts,
            boot_count,
        })
    }
}

/// An error returned when a state record cannot be parsed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum StateError {
    /// The stored CRC did not match the contents.
    BadCrc,
    /// The record format version is not [`STATE_FORMAT_VERSION`].
    UnsupportedFormat(u8),
    /// A field held an unknown value.
    BadField,
}

impl fmt::Display for StateError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::BadCrc => f.write_str("state CRC mismatch"),
            Self::UnsupportedFormat(v) => write!(f, "unsupported state format version {v}"),
            Self::BadField => f.write_str("state field out of range"),
        }
    }
}

impl core::error::Error for StateError {}

#[cfg(test)]
mod tests {
    use cellboot::image::Region;

    use super::{AppHealth, PersistentState, STATE_LEN, StagedState, StateError, UpdateOutcome};

    fn sample() -> PersistentState {
        PersistentState {
            agent_version: 0x0102_0304,
            app_version: 7,
            staged_version: 8,
            app_health: AppHealth::Good,
            staged: StagedState::Ready,
            staged_region: Some(Region::ApplicationCode),
            last_outcome: UpdateOutcome::Success,
            program_attempts: 0,
            boot_count: 3,
        }
    }

    #[test]
    fn roundtrip() {
        let state = sample();
        assert_eq!(PersistentState::parse(&state.serialize()), Ok(state));
    }

    #[test]
    fn program_attempts_and_outcome_roundtrip() {
        let state = PersistentState {
            program_attempts: 2,
            last_outcome: UpdateOutcome::ProgramFailed,
            ..sample()
        };
        assert_eq!(PersistentState::parse(&state.serialize()), Ok(state));
    }

    #[test]
    fn fresh_roundtrip() {
        let state = PersistentState::new(1);
        let parsed = PersistentState::parse(&state.serialize()).unwrap();
        assert_eq!(parsed, state);
        assert_eq!(parsed.staged_region, None);
    }

    #[test]
    fn detects_corruption() {
        let mut bytes = sample().serialize();
        bytes[8] ^= 0x01;
        assert_eq!(PersistentState::parse(&bytes), Err(StateError::BadCrc));
    }

    #[test]
    fn detects_bad_format() {
        let mut bytes = PersistentState::new(1).serialize();
        bytes[0] = 9;
        // Recompute the CRC so the format check is what trips, not the CRC.
        let crc = crc::checksum32(&bytes[0..24]);
        bytes[24..28].copy_from_slice(&crc.to_le_bytes());
        assert_eq!(
            PersistentState::parse(&bytes),
            Err(StateError::UnsupportedFormat(9))
        );
    }

    #[test]
    fn len_is_stable() {
        assert_eq!(PersistentState::new(0).serialize().len(), STATE_LEN);
    }

    #[test]
    fn mark_programmed_advances_app_version_and_resets_health() {
        let mut state = PersistentState {
            staged: StagedState::Ready,
            staged_region: Some(Region::ApplicationCode),
            staged_version: 42,
            app_version: 7,
            app_health: AppHealth::Good,
            program_attempts: 2,
            boot_count: 5,
            ..sample()
        };
        state.mark_programmed(Region::ApplicationCode);
        assert_eq!(state.staged, StagedState::Empty);
        assert_eq!(state.staged_region, None);
        assert_eq!(state.last_outcome, UpdateOutcome::Success);
        assert_eq!(state.program_attempts, 0);
        assert_eq!(state.app_version, 42);
        assert_eq!(state.app_health, AppHealth::Unknown);
        assert_eq!(state.boot_count, 0);
    }

    #[test]
    fn mark_programmed_non_app_keeps_app_version_and_health() {
        let mut state = PersistentState {
            staged: StagedState::Ready,
            staged_region: Some(Region::Bootloader),
            staged_version: 42,
            app_version: 7,
            app_health: AppHealth::Good,
            program_attempts: 1,
            boot_count: 3,
            ..sample()
        };
        state.mark_programmed(Region::Bootloader);
        assert_eq!(state.last_outcome, UpdateOutcome::Success);
        // A bootloader flash does not touch the recorded app version/health.
        assert_eq!(state.app_version, 7);
        assert_eq!(state.app_health, AppHealth::Good);
        assert_eq!(state.boot_count, 3);
    }

    #[test]
    fn mark_program_failed_clears_slot_and_records_failure() {
        let mut state = PersistentState {
            staged: StagedState::Ready,
            staged_region: Some(Region::ApplicationCode),
            program_attempts: 3,
            ..sample()
        };
        state.mark_program_failed();
        assert_eq!(state.staged, StagedState::Empty);
        assert_eq!(state.staged_region, None);
        assert_eq!(state.last_outcome, UpdateOutcome::ProgramFailed);
        assert_eq!(state.program_attempts, 0);
    }
}
