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

/// A writer for a target's non-volatile program memory.
///
/// On `CellGuard` this is implemented by the `ATtiny406` programmer over UPDI. The
/// AVR128 never programs its own flash.
pub trait NvmWriter {
    /// Error type reported by the writer.
    type Error;

    /// Size of a program-memory page in bytes.
    ///
    /// Addresses passed to [`NvmWriter::write_page`] must be page-aligned and
    /// `data` must be exactly this long.
    fn page_size(&self) -> usize;

    /// Erases and writes one page of program memory at `address`.
    ///
    /// # Errors
    ///
    /// Returns an error if the address is misaligned, the length is wrong, or
    /// the write fails.
    fn write_page(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error>;

    /// Reads `buf.len()` bytes of program memory starting at `address`.
    ///
    /// Used to read the target back for verification.
    ///
    /// # Errors
    ///
    /// Returns an error if the range is out of bounds or the read fails.
    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Self::Error>;
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
