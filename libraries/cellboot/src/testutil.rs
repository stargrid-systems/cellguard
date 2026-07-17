//! Shared test utilities for downstream crates.
//!
//! Simple in-RAM implementations of [`ImageStore`] and [`StateStore`] that
//! several test suites need. Gated behind the `testutil` feature so they do
//! not ship in production builds.
//!
//! Enable with `cellboot = { workspace = true, features = ["testutil"] }` in
//! `[dev-dependencies]`.

use core::cell::RefCell;

use crate::io::{ImageStore, StateStore};

/// A fixed-capacity in-RAM [`ImageStore`] for tests.
///
/// The buffer starts zeroed and is publicly readable so a test can inspect
/// what was written. `N` is the capacity in bytes.
pub struct MemStore<const N: usize> {
    /// Backing storage, exposed for test inspection.
    pub buf: [u8; N],
}

impl<const N: usize> MemStore<N> {
    /// Creates a zeroed store.
    #[must_use]
    pub const fn new() -> Self {
        Self { buf: [0; N] }
    }
}

impl<const N: usize> Default for MemStore<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Error returned by [`MemStore`] when a range leaves the buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct OutOfBounds;

impl core::fmt::Display for OutOfBounds {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("MemStore access out of bounds")
    }
}

impl core::error::Error for OutOfBounds {}

impl<const N: usize> ImageStore for MemStore<N> {
    type Error = OutOfBounds;

    fn capacity(&self) -> u32 {
        u32::try_from(N).unwrap_or(u32::MAX)
    }

    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), OutOfBounds> {
        let start = usize::try_from(offset).map_err(|_| OutOfBounds)?;
        let end = start.checked_add(buf.len()).ok_or(OutOfBounds)?;
        buf.copy_from_slice(self.buf.get(start..end).ok_or(OutOfBounds)?);
        Ok(())
    }

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), OutOfBounds> {
        let start = usize::try_from(offset).map_err(|_| OutOfBounds)?;
        let end = start.checked_add(data.len()).ok_or(OutOfBounds)?;
        self.buf
            .get_mut(start..end)
            .ok_or(OutOfBounds)?
            .copy_from_slice(data);
        Ok(())
    }
}

/// A [`StateStore`] that always reports an empty load and drops writes.
///
/// Useful for tests that need an agent with no persisted state.
#[derive(Debug, Default)]
pub struct NullStateStore;

impl StateStore for NullStateStore {
    type Error = ();

    fn load(&mut self, _buf: &mut [u8]) -> Result<(), ()> {
        Err(())
    }

    fn store(&mut self, _data: &[u8]) -> Result<(), ()> {
        Ok(())
    }
}

/// A [`StateStore`] backed by shared bytes, so a second agent can load what a
/// first one persisted.
///
/// `None` stands in for a blank store. The caller owns the backing
/// [`RefCell`] and passes a reference, so two agents constructed in sequence
/// (simulating a reset) can share it.
pub struct SharedStore<'a, const L: usize> {
    backing: &'a RefCell<Option<[u8; L]>>,
}

impl<'a, const L: usize> SharedStore<'a, L> {
    /// Wraps a shared backing cell.
    #[must_use]
    pub const fn new(backing: &'a RefCell<Option<[u8; L]>>) -> Self {
        Self { backing }
    }
}

impl<const L: usize> StateStore for SharedStore<'_, L> {
    type Error = ();

    fn load(&mut self, buf: &mut [u8]) -> Result<(), ()> {
        let guard = self.backing.borrow();
        let bytes = guard.as_ref().ok_or(())?;
        buf.get_mut(..L).ok_or(())?.copy_from_slice(bytes);
        Ok(())
    }

    fn store(&mut self, data: &[u8]) -> Result<(), ()> {
        let bytes: [u8; L] = data.get(..L).ok_or(())?.try_into().map_err(|_| ())?;
        *self.backing.borrow_mut() = Some(bytes);
        Ok(())
    }
}
