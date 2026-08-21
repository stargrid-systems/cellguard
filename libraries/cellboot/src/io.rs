//! The I/O traits through which a target performs all input and output.
//!
//! The core logic is written against these traits only. Each target supplies
//! concrete implementations.

/// Byte-addressable staging storage for a firmware image.
///
/// On `CellGuard` this is the external SPI EEPROM: the AVR128 stages a
/// received image here and the `ATtiny406` reads it back.
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
/// Streams so a 256-byte programmer can push a 512-byte-page target: writes
/// arrive as sequential, sub-page chunks and the implementation handles the
/// target's page mechanics.
pub trait NvmWriter {
    /// Error type reported by the writer.
    type Error;

    /// Enters programming mode (halting the target) and erases the program
    /// memory to be written.
    ///
    /// # Errors
    ///
    /// Returns an error if the target cannot be entered or erased.
    fn begin(&mut self) -> Result<(), Self::Error>;

    /// Writes `data` at `address`, extending the previous write.
    ///
    /// Chunks arrive in ascending, contiguous order starting from a page
    /// boundary.
    ///
    /// # Errors
    ///
    /// Returns an error if the write fails.
    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error>;

    /// Reads `buf.len()` bytes back from `address` for verification.
    ///
    /// Any page still buffered from [`NvmWriter::write`] is committed first,
    /// so the read reflects final flash contents.
    ///
    /// # Errors
    ///
    /// Returns an error if the read fails.
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Self::Error>;

    /// Ends the session: commits any buffered page and lets the target run.
    ///
    /// # Errors
    ///
    /// Returns an error if the final commit fails.
    fn finish(&mut self) -> Result<(), Self::Error>;
}

/// Persistent storage for the updater's own state.
///
/// The state must survive a program-memory rewrite, so on-chip EEPROM is the
/// natural backing.
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

/// Persistent storage for the shared authentication key.
///
/// The key lives in the AVR128 USERROW, provisioned at the factory. Only
/// development builds provide a writable implementation.
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
/// The production default: the key can never be replaced over the bus.
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
/// Offsets below `low`'s capacity map to `low`. Higher offsets map to `high`
/// rebased. An access that straddles the boundary is rejected.
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

/// A paged flash target that supports page-erase and chunk-write.
///
/// Driven by the shared [`write_with_page_erase`] helper, so the
/// page-erase-and-split loop exists in one place.
pub trait PagedFlash {
    /// Error type reported by the target.
    type Error;
    /// Erases the page containing `page_base` (a page-aligned address).
    ///
    /// # Errors
    ///
    /// Returns the target's error if the erase fails.
    fn erase_page(&mut self, page_base: u32) -> Result<(), Self::Error>;
    /// Writes `chunk` at `addr`. The page containing `addr` must already be
    /// erased.
    ///
    /// # Errors
    ///
    /// Returns the target's error if the write fails.
    fn write_chunk(&mut self, addr: u32, chunk: &[u8]) -> Result<(), Self::Error>;
}

/// Streams `data` to a page-oriented target, erasing each page the first time
/// it is touched.
///
/// Assumes writes arrive in ascending, contiguous order from a page boundary,
/// and splits chunks at page boundaries so the caller never buffers a whole
/// page. `erased_page` advances only after a successful erase, so a mid-stream
/// failure re-erases the in-flight page on the next call.
///
/// # Errors
///
/// Returns the target's error if any erase or write fails.
pub fn write_with_page_erase<T: PagedFlash>(
    address: u32,
    data: &[u8],
    page_size: u32,
    erased_page: &mut Option<u32>,
    target: &mut T,
) -> Result<(), T::Error> {
    let mut addr = address;
    let mut rest = data;
    while !rest.is_empty() {
        let page = addr / page_size;
        if *erased_page != Some(page) {
            target.erase_page(page.saturating_mul(page_size))?;
            *erased_page = Some(page);
        }
        let page_end = page.saturating_add(1).saturating_mul(page_size);
        let room = usize::try_from(page_end.saturating_sub(addr)).unwrap_or(usize::MAX);
        let n = rest.len().min(room);
        let (chunk, tail) = rest.split_at(n);
        target.write_chunk(addr, chunk)?;
        addr = addr.saturating_add(u32::try_from(chunk.len()).unwrap_or(u32::MAX));
        rest = tail;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use crate::io::{BandedError, BandedStore, ImageStore};
    use crate::testutil::MemStore;

    #[test]
    fn banded_store_routes_and_rejects_straddles() {
        let mut store = BandedStore::new(MemStore::<64>::new(), MemStore::<32>::new());
        assert_eq!(store.capacity(), 96);

        store.write(10, &[0xAA; 4]).unwrap();
        store.write(64, &[0xBB; 4]).unwrap();

        let mut buf = [0u8; 4];
        store.read(10, &mut buf).unwrap();
        assert_eq!(buf, [0xAA; 4]);
        store.read(64, &mut buf).unwrap();
        assert_eq!(buf, [0xBB; 4]);

        assert_eq!(store.write(62, &[0; 4]), Err(BandedError::OutOfBounds));
    }
}
