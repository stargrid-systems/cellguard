//! The I/O traits through which a target performs all input and output.
//!
//! The core logic is written against these traits only. Each target provides
//! concrete implementations: the AVR128 backs [`ImageStore`] with the external
//! EEPROM driver, the `ATtiny406` programmer backs [`NvmWriter`] with its UPDI
//! writer, and so on. Nothing here touches a register.

/// Byte-addressable staging storage for a firmware image.
///
/// On `CellGuard` this is the external SPI EEPROM. The AVR128 writes a received
/// image here and the `ATtiny406` reads it back to program the target.
pub trait ImageStore {
    /// Error type reported by the backing storage.
    type Error;

    /// Total capacity in bytes.
    fn capacity(&self) -> u32;

    /// Reads `buf.len()` bytes starting at `offset`.
    ///
    /// # Errors
    ///
    /// Returns an error if the range is out of bounds or the backing storage
    /// fails.
    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Writes `data` starting at `offset`.
    ///
    /// # Errors
    ///
    /// Returns an error if the range is out of bounds or the backing storage
    /// fails.
    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error>;
}

/// A streaming writer for a target's non-volatile program memory.
///
/// On `CellGuard` this is implemented by the `ATtiny406` programmer over UPDI,
/// so the AVR128 never programs its own flash. It streams so the 256-byte
/// programmer can push a 512-byte-page target: [`NvmWriter::write`] is called
/// with sequential, sub-page chunks and the implementation handles the target's
/// page mechanics.
pub trait NvmWriter {
    /// Error type reported by the writer.
    type Error;

    /// Begins a programming session: enters programming mode (halting the
    /// target) and erases the program memory to be written.
    ///
    /// # Errors
    ///
    /// Returns an error if the target cannot be entered or erased.
    fn begin(&mut self) -> Result<(), Self::Error>;

    /// Writes `data` at `address`, extending the previous write.
    ///
    /// Chunks arrive in ascending, contiguous order starting from a page
    /// boundary. The implementation buffers into the target's page and commits
    /// full pages as they fill.
    ///
    /// # Errors
    ///
    /// Returns an error if the write fails.
    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error>;

    /// Reads `buf.len()` bytes back from `address` for verification.
    ///
    /// Any page still buffered from [`NvmWriter::write`] is committed first, so
    /// a read always reflects what will be in flash.
    ///
    /// # Errors
    ///
    /// Returns an error if the read fails.
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Ends the session: commits any buffered page, leaves programming mode,
    /// and lets the target run.
    ///
    /// # Errors
    ///
    /// Returns an error if the final commit or release fails.
    fn finish(&mut self) -> Result<(), Self::Error>;
}

/// Persistent storage for the updater's own state.
///
/// This holds the probe-able status: which image is current, whether it is
/// valid, boot counters, and the last error. It survives a program-memory
/// rewrite, so on-chip EEPROM is the natural backing.
pub trait StateStore {
    /// Error type reported by the store.
    type Error;

    /// Loads the stored state into `buf`.
    ///
    /// # Errors
    ///
    /// Returns an error if the read fails.
    fn load(&mut self, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Persists `data` as the new state.
    ///
    /// # Errors
    ///
    /// Returns an error if the write fails.
    fn store(&mut self, data: &[u8]) -> Result<(), Self::Error>;
}

/// Control over reset and the watchdog.
pub trait SystemControl {
    /// Services the watchdog so a long operation does not trip it.
    fn service_watchdog(&mut self);

    /// Resets the system. Does not return.
    fn reset(&mut self) -> !;
}

/// Persistent storage for the shared authentication key.
///
/// The key normally lives in the AVR128 USERROW, provisioned once at the
/// factory. Only development builds provide a writable implementation; see
/// [`NoKeyStore`] for the production default.
pub trait KeyStore {
    /// Error type reported by the store.
    type Error;

    /// Persists `key` as the new authentication key.
    ///
    /// # Errors
    ///
    /// Returns an error if the store is locked or the write fails.
    fn write_key(&mut self, key: &[u8]) -> Result<(), Self::Error>;
}

/// A [`KeyStore`] that rejects every write.
///
/// This is the production default: the key is locked and can never be replaced
/// over the bus, so a `BootReplaceKey` command is answered with a rejection.
#[derive(Debug, Clone, Copy, Default)]
pub struct NoKeyStore;

impl KeyStore for NoKeyStore {
    type Error = KeyLocked;

    fn write_key(&mut self, _key: &[u8]) -> Result<(), Self::Error> {
        Err(KeyLocked)
    }
}

/// The error returned by [`NoKeyStore`]: key replacement is disabled.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct KeyLocked;

impl core::fmt::Display for KeyLocked {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str("key replacement is disabled")
    }
}

impl core::error::Error for KeyLocked {}

/// An [`ImageStore`] that bands two stores into one address space.
///
/// The `CellGuard` board stages each firmware region on its own SPI EEPROM
/// chip. A banded store maps the low band (offsets below `low`'s capacity) to
/// `low` and the high band to `high` with a rebased offset. An access that
/// straddles the boundary is rejected. Both the cellcore and the cellprog
/// programmer use this to address their App and Boot EEPROMs as one store.
pub struct BandedStore<A, B> {
    low: A,
    high: B,
    split: u32,
}

impl<A: ImageStore, B: ImageStore> BandedStore<A, B> {
    /// Bands `high` after `low`. The boundary is `low`'s capacity.
    #[must_use]
    pub fn new(low: A, high: B) -> Self {
        let split = low.capacity();
        Self { low, high, split }
    }

    /// Resolves an access to a band and a rebased offset within it.
    fn locate(&self, offset: u32, len: usize) -> Result<Band, BandedError<A::Error, B::Error>> {
        let len = u32::try_from(len).map_err(|_| BandedError::OutOfBounds)?;
        let end = offset.checked_add(len).ok_or(BandedError::OutOfBounds)?;
        if end <= self.split {
            Ok(Band::Low(offset))
        } else if offset >= self.split {
            Ok(Band::High(offset - self.split))
        } else {
            Err(BandedError::OutOfBounds)
        }
    }
}

enum Band {
    Low(u32),
    High(u32),
}

impl<A: ImageStore, B: ImageStore> ImageStore for BandedStore<A, B> {
    type Error = BandedError<A::Error, B::Error>;

    fn capacity(&self) -> u32 {
        self.split.saturating_add(self.high.capacity())
    }

    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        match self.locate(offset, buf.len())? {
            Band::Low(at) => self.low.read(at, buf).map_err(BandedError::Low),
            Band::High(at) => self.high.read(at, buf).map_err(BandedError::High),
        }
    }

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), Self::Error> {
        match self.locate(offset, data.len())? {
            Band::Low(at) => self.low.write(at, data).map_err(BandedError::Low),
            Band::High(at) => self.high.write(at, data).map_err(BandedError::High),
        }
    }
}

/// The error returned by [`BandedStore`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BandedError<L, H> {
    /// The low band's store failed.
    Low(L),
    /// The high band's store failed.
    High(H),
    /// The access was out of range or straddled the band boundary.
    OutOfBounds,
}

impl<L: core::fmt::Display, H: core::fmt::Display> core::fmt::Display for BandedError<L, H> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Low(e) => write!(f, "low band: {e}"),
            Self::High(e) => write!(f, "high band: {e}"),
            Self::OutOfBounds => f.write_str("banded store access out of bounds"),
        }
    }
}

impl<L, H> core::error::Error for BandedError<L, H>
where
    L: core::error::Error,
    H: core::error::Error,
{
}
