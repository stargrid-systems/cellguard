//! Driver for the ON Semiconductor CAT25 family of SPI EEPROMs.
//!
//! [`Cat25`] is a blocking client. Every write enables the device, issues the
//! command, and waits for the self-timed write cycle to finish. There is no way
//! to issue a half-finished write sequence, so the usual protocol mistakes
//! cannot happen.

#![no_std]
#![warn(missing_docs)]

use embedded_hal::delay::DelayNs;
use embedded_hal::spi::{Operation, SpiDevice};

use self::command::{HEADER_MAX, PageChunks, encode_header, range_in_bounds};
pub use self::error::Error;
pub use self::model::{CAT25M01, CAT25128, Model};
pub use self::register::{BlockProtection, Status};

mod command;
mod error;
mod model;
mod register;

/// Number of times a write polls the status register before giving up.
const WRITE_POLL_ATTEMPTS: u32 = 10;

/// Delay between status register polls in microseconds.
///
/// The worst case write cycle (tWC) is 5 ms, so the total budget of 10 ms
/// leaves a 2x margin.
const WRITE_POLL_INTERVAL_US: u32 = 1_000;

/// A CAT25 family SPI EEPROM.
///
/// Holds the SPI device and a delay provider. Writes block until the write
/// cycle finishes, polling the status register and spacing the polls with the
/// delay.
pub struct Cat25<S, D> {
    spi: S,
    delay: D,
    model: Model,
}

impl<S: SpiDevice, D: DelayNs> Cat25<S, D> {
    /// Creates a driver for a specific model.
    pub const fn new(spi: S, model: Model, delay: D) -> Self {
        Self { spi, delay, model }
    }

    /// Releases the SPI device and delay provider.
    pub fn into_parts(self) -> (S, D) {
        (self.spi, self.delay)
    }

    /// Returns the model this driver was created for.
    pub const fn model(&self) -> Model {
        self.model
    }

    /// Reads the status register (RDSR).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Spi`] if the SPI transaction fails.
    pub fn status(&mut self) -> Result<Status, Error<S::Error>> {
        self.read_status().map_err(Error::Spi)
    }

    /// Clears the write enable latch (WRDI).
    ///
    /// You rarely need this. The driver enables writes right before each write,
    /// and the device clears the latch on its own once a write cycle finishes.
    /// So a successful write never leaves it set. Reads do not touch it.
    ///
    /// The one exception is a rejected write. When a write is blocked (see
    /// [`Error::WriteProtected`]) no write cycle runs, so the latch stays set.
    /// Call this to clear it defensively after such a failure.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Spi`] if the SPI transaction fails.
    pub fn write_disable(&mut self) -> Result<(), Error<S::Error>> {
        self.spi.write(&[command::WRDI]).map_err(Error::Spi)
    }

    /// Reads `data.len()` bytes from the main array starting at `address`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::OutOfBounds`] if the range does not fit in the device,
    /// or [`Error::Spi`] if the SPI transaction fails.
    pub fn read(&mut self, address: u32, data: &mut [u8]) -> Result<(), Error<S::Error>> {
        if !range_in_bounds(address, data.len(), self.model.size()) {
            return Err(Error::OutOfBounds);
        }
        self.read_memory(address, data).map_err(Error::Spi)
    }

    /// Writes `data` to the main array starting at `address`.
    ///
    /// The write is split at page boundaries so it may span pages.
    ///
    /// # Errors
    ///
    /// Returns [`Error::OutOfBounds`] if the range does not fit in the device,
    /// [`Error::WriteProtected`] if the device rejects a write,
    /// [`Error::Timeout`] if a write cycle does not finish in time, or
    /// [`Error::Spi`] if a SPI transaction fails.
    pub fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Error<S::Error>> {
        if !range_in_bounds(address, data.len(), self.model.size()) {
            return Err(Error::OutOfBounds);
        }
        let mut rest = data;
        for (addr, len) in PageChunks::new(address, data.len(), self.model.page_size()) {
            let (head, tail) = rest.split_at(len);
            self.write_page(addr, head)?;
            rest = tail;
        }
        Ok(())
    }

    /// Reads `data.len()` bytes from the identification page starting at
    /// `offset`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::OutOfBounds`] if the range does not fit in the
    /// identification page, [`Error::WriteProtected`] if the device rejects the
    /// status write that selects the page, [`Error::Timeout`] if that write
    /// does not finish in time, or [`Error::Spi`] if a SPI transaction fails.
    pub fn read_id_page(&mut self, offset: u32, data: &mut [u8]) -> Result<(), Error<S::Error>> {
        if data.is_empty() {
            return Ok(());
        }
        if !range_in_bounds(offset, data.len(), u32::from(self.model.page_size())) {
            return Err(Error::OutOfBounds);
        }
        self.select_id_page()?;
        // The read targets the identification page and clears the latch.
        self.read_memory(offset, data).map_err(Error::Spi)
    }

    /// Writes `data` to the identification page starting at `offset`.
    ///
    /// The identification page is a single page, so the write cannot span page
    /// boundaries.
    ///
    /// # Errors
    ///
    /// Returns [`Error::OutOfBounds`] if the range does not fit in the
    /// identification page, [`Error::WriteProtected`] if the device rejects a
    /// write, [`Error::Timeout`] if a write cycle does not finish in time, or
    /// [`Error::Spi`] if a SPI transaction fails.
    pub fn write_id_page(&mut self, offset: u32, data: &[u8]) -> Result<(), Error<S::Error>> {
        if data.is_empty() {
            return Ok(());
        }
        if !range_in_bounds(offset, data.len(), u32::from(self.model.page_size())) {
            return Err(Error::OutOfBounds);
        }
        self.select_id_page()?;
        // Selecting the page cleared write enable, so the page write re-enables.
        self.write_page(offset, data)
    }

    /// Permanently locks the identification page in read-only mode.
    ///
    /// This is irreversible. The identification page can never be written again
    /// after this call succeeds.
    ///
    /// # Errors
    ///
    /// Returns [`Error::WriteProtected`] if the device rejects the status
    /// write, [`Error::Timeout`] if it does not finish in time, or
    /// [`Error::Spi`] if a SPI transaction fails.
    pub fn lock_id_page(&mut self) -> Result<(), Error<S::Error>> {
        self.modify_status(|s| s.with_lock_id_page(true))
    }

    /// Sets the block protection level of the main array.
    ///
    /// # Errors
    ///
    /// Returns [`Error::WriteProtected`] if the device rejects the status
    /// write, [`Error::Timeout`] if it does not finish in time, or
    /// [`Error::Spi`] if a SPI transaction fails.
    pub fn set_block_protection(
        &mut self,
        protection: BlockProtection,
    ) -> Result<(), Error<S::Error>> {
        self.modify_status(|s| s.with_block_protection(protection))
    }

    /// Enables or disables the write protect pin (the WPEN bit).
    ///
    /// With WPEN set and the WP pin held low, status register writes are
    /// blocked in hardware. A later status change then returns
    /// [`Error::WriteProtected`].
    ///
    /// # Errors
    ///
    /// Returns [`Error::WriteProtected`] if the device rejects the status
    /// write, [`Error::Timeout`] if it does not finish in time, or
    /// [`Error::Spi`] if a SPI transaction fails.
    pub fn set_write_protect_enabled(&mut self, enable: bool) -> Result<(), Error<S::Error>> {
        self.modify_status(|s| s.with_write_protect_enabled(enable))
    }

    /// Selects the identification page for the next read or write.
    ///
    /// The latch stays set until the next read or write, which the device then
    /// directs to the identification page. The lock bit is left untouched, so a
    /// locked page stays locked and still readable.
    fn select_id_page(&mut self) -> Result<(), Error<S::Error>> {
        self.modify_status(|s| s.with_id_page_latch(true))
    }

    /// Reads the current status, applies `change`, and writes it back.
    ///
    /// Only the writable bits are carried over, so a change preserves the bits
    /// it does not touch instead of clobbering them.
    fn modify_status(
        &mut self,
        change: impl FnOnce(Status) -> Status,
    ) -> Result<(), Error<S::Error>> {
        let current = self.read_status().map_err(Error::Spi)?.writable();
        self.write_status_register(change(current))
    }

    /// Enables writes, writes the status register, and waits for the cycle.
    fn write_status_register(&mut self, status: Status) -> Result<(), Error<S::Error>> {
        self.write_enable().map_err(Error::Spi)?;
        self.spi
            .write(&[command::WRSR, status.bits()])
            .map_err(Error::Spi)?;
        self.wait_written()
    }

    /// Enables writes, writes one page, and waits for the cycle.
    fn write_page(&mut self, address: u32, data: &[u8]) -> Result<(), Error<S::Error>> {
        if data.is_empty() {
            return Ok(());
        }
        self.write_enable().map_err(Error::Spi)?;
        let mut buf = [0u8; HEADER_MAX];
        let header = encode_header(self.model, &mut buf, command::WRITE, address);
        self.spi
            .transaction(&mut [Operation::Write(header), Operation::Write(data)])
            .map_err(Error::Spi)?;
        self.wait_written()
    }

    fn write_enable(&mut self) -> Result<(), S::Error> {
        self.spi.write(&[command::WREN])
    }

    fn read_memory(&mut self, address: u32, data: &mut [u8]) -> Result<(), S::Error> {
        if data.is_empty() {
            return Ok(());
        }
        let mut buf = [0u8; HEADER_MAX];
        let header = encode_header(self.model, &mut buf, command::READ, address);
        self.spi
            .transaction(&mut [Operation::Write(header), Operation::Read(data)])
    }

    fn read_status(&mut self) -> Result<Status, S::Error> {
        let mut buf = [0u8; 1];
        self.spi.transaction(&mut [
            Operation::Write(&[command::RDSR]),
            Operation::Read(&mut buf),
        ])?;
        Ok(Status::from_bits(buf[0]))
    }

    /// Waits for the current write cycle to finish and confirms it ran.
    ///
    /// The device clears the write enable latch only when it actually performs
    /// a write. A rejected write leaves the latch set, which surfaces as
    /// [`Error::WriteProtected`].
    fn wait_written(&mut self) -> Result<(), Error<S::Error>> {
        for _ in 0..WRITE_POLL_ATTEMPTS {
            self.delay.delay_us(WRITE_POLL_INTERVAL_US);
            let status = self.read_status().map_err(Error::Spi)?;
            if status.ready() {
                return if status.write_enabled() {
                    Err(Error::WriteProtected)
                } else {
                    Ok(())
                };
            }
        }
        Err(Error::Timeout)
    }
}

#[cfg(test)]
mod tests {
    extern crate std;

    use std::vec;
    use std::vec::Vec;

    use embedded_hal_mock::eh1::delay::NoopDelay;
    use embedded_hal_mock::eh1::spi::{Mock as SpiMock, Transaction as SpiTransaction};

    use super::*;

    type Spi = SpiMock<u8>;

    fn build(model: Model, expectations: &[SpiTransaction<u8>]) -> Cat25<Spi, NoopDelay> {
        Cat25::new(SpiMock::new(expectations), model, NoopDelay::new())
    }

    fn finish(cat: Cat25<Spi, NoopDelay>) {
        let (mut spi, _delay) = cat.into_parts();
        spi.done();
    }

    fn wren() -> [SpiTransaction<u8>; 3] {
        [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x06]),
            SpiTransaction::transaction_end(),
        ]
    }

    fn rdsr(value: u8) -> [SpiTransaction<u8>; 4] {
        [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x05]),
            SpiTransaction::read_vec(vec![value]),
            SpiTransaction::transaction_end(),
        ]
    }

    fn wrsr(value: u8) -> [SpiTransaction<u8>; 3] {
        [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x01, value]),
            SpiTransaction::transaction_end(),
        ]
    }

    fn cmd(header: Vec<u8>, data: Vec<u8>) -> [SpiTransaction<u8>; 4] {
        [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(header),
            SpiTransaction::write_vec(data),
            SpiTransaction::transaction_end(),
        ]
    }

    #[test]
    fn status_reads_rdsr() {
        let mut cat = build(CAT25128, &rdsr(0b0000_0010));
        let status = cat.status().unwrap();
        assert!(status.ready());
        assert!(status.write_enabled());
        finish(cat);
    }

    #[test]
    fn write_disable_sends_wrdi() {
        let expected = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x04]),
            SpiTransaction::transaction_end(),
        ];
        let mut cat = build(CAT25128, &expected);
        cat.write_disable().unwrap();
        finish(cat);
    }

    #[test]
    fn read_sends_opcode_and_address() {
        let expected = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x03, 0x12, 0x34]),
            SpiTransaction::read_vec(vec![0xAA, 0xBB]),
            SpiTransaction::transaction_end(),
        ];
        let mut cat = build(CAT25128, &expected);
        let mut buf = [0u8; 2];
        cat.read(0x1234, &mut buf).unwrap();
        assert_eq!(buf, [0xAA, 0xBB]);
        finish(cat);
    }

    #[test]
    fn m01_uses_three_address_bytes() {
        let expected = [
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x03, 0x01, 0x23, 0x45]),
            SpiTransaction::read_vec(vec![0x00]),
            SpiTransaction::transaction_end(),
        ];
        let mut cat = build(CAT25M01, &expected);
        let mut buf = [0u8; 1];
        cat.read(0x01_2345, &mut buf).unwrap();
        finish(cat);
    }

    #[test]
    fn write_enables_writes_and_waits() {
        let mut expected = Vec::new();
        expected.extend(wren());
        expected.extend(cmd(vec![0x02, 0x00, 0x10], vec![0xDE, 0xAD]));
        expected.extend(rdsr(0x00));

        let mut cat = build(CAT25128, &expected);
        cat.write(0x0010, &[0xDE, 0xAD]).unwrap();
        finish(cat);
    }

    #[test]
    fn write_splits_across_page_boundary() {
        let mut expected = Vec::new();
        // Page size 64. Starting at 63 with 3 bytes spans two pages.
        expected.extend(wren());
        expected.extend(cmd(vec![0x02, 0x00, 0x3F], vec![0xA0]));
        expected.extend(rdsr(0x00));
        expected.extend(wren());
        expected.extend(cmd(vec![0x02, 0x00, 0x40], vec![0xA1, 0xA2]));
        expected.extend(rdsr(0x00));

        let mut cat = build(CAT25128, &expected);
        cat.write(63, &[0xA0, 0xA1, 0xA2]).unwrap();
        finish(cat);
    }

    #[test]
    fn write_polls_until_ready() {
        let mut expected = Vec::new();
        expected.extend(wren());
        expected.extend(cmd(vec![0x02, 0x00, 0x00], vec![0x01]));
        expected.extend(rdsr(0b0000_0001)); // busy
        expected.extend(rdsr(0x00)); // ready, write enable cleared

        let mut cat = build(CAT25128, &expected);
        cat.write(0, &[0x01]).unwrap();
        finish(cat);
    }

    #[test]
    fn write_rejected_reports_protected() {
        let mut expected = Vec::new();
        expected.extend(wren());
        expected.extend(cmd(vec![0x02, 0x00, 0x00], vec![0x01]));
        // Ready but write enable still set means the write did not run.
        expected.extend(rdsr(0b0000_0010));

        let mut cat = build(CAT25128, &expected);
        assert_eq!(cat.write(0, &[0x01]), Err(Error::WriteProtected));
        finish(cat);
    }

    #[test]
    fn write_times_out_when_never_ready() {
        let mut expected = Vec::new();
        expected.extend(wren());
        expected.extend(cmd(vec![0x02, 0x00, 0x00], vec![0x01]));
        for _ in 0..WRITE_POLL_ATTEMPTS {
            expected.extend(rdsr(0b0000_0001)); // always busy
        }

        let mut cat = build(CAT25128, &expected);
        assert_eq!(cat.write(0, &[0x01]), Err(Error::Timeout));
        finish(cat);
    }

    #[test]
    fn read_id_page_selects_then_reads() {
        let mut expected = Vec::new();
        expected.extend(rdsr(0x00)); // read current status for the modify
        expected.extend(wren());
        expected.extend(wrsr(0x40));
        expected.extend(rdsr(0x00));
        expected.extend([
            SpiTransaction::transaction_start(),
            SpiTransaction::write_vec(vec![0x03, 0x00, 0x00]),
            SpiTransaction::read_vec(vec![0x01, 0x02, 0x03, 0x04]),
            SpiTransaction::transaction_end(),
        ]);

        let mut cat = build(CAT25128, &expected);
        let mut buf = [0u8; 4];
        cat.read_id_page(0, &mut buf).unwrap();
        assert_eq!(buf, [0x01, 0x02, 0x03, 0x04]);
        finish(cat);
    }

    #[test]
    fn write_id_page_selects_then_writes() {
        let mut expected = Vec::new();
        expected.extend(rdsr(0x00)); // read current status for the modify
        expected.extend(wren());
        expected.extend(wrsr(0x40));
        expected.extend(rdsr(0x00));
        expected.extend(wren());
        expected.extend(cmd(vec![0x02, 0x00, 0x05], vec![0x11, 0x22]));
        expected.extend(rdsr(0x00));

        let mut cat = build(CAT25128, &expected);
        cat.write_id_page(5, &[0x11, 0x22]).unwrap();
        finish(cat);
    }

    #[test]
    fn lock_id_page_sets_lip_bit() {
        let mut expected = Vec::new();
        expected.extend(rdsr(0x00)); // read current status for the modify
        expected.extend(wren());
        expected.extend(wrsr(0x10));
        expected.extend(rdsr(0x00));

        let mut cat = build(CAT25128, &expected);
        cat.lock_id_page().unwrap();
        finish(cat);
    }

    #[test]
    fn set_block_protection_writes_status() {
        let mut expected = Vec::new();
        expected.extend(rdsr(0x00)); // read current status for the modify
        expected.extend(wren());
        expected.extend(wrsr(0b0000_1100)); // BlockProtection::All
        expected.extend(rdsr(0x00));

        let mut cat = build(CAT25128, &expected);
        cat.set_block_protection(BlockProtection::All).unwrap();
        finish(cat);
    }

    #[test]
    fn set_write_protect_enabled_writes_status() {
        let mut expected = Vec::new();
        expected.extend(rdsr(0x00));
        expected.extend(wren());
        expected.extend(wrsr(0b1000_0000)); // WPEN
        expected.extend(rdsr(0x00));

        let mut cat = build(CAT25128, &expected);
        cat.set_write_protect_enabled(true).unwrap();
        finish(cat);
    }

    #[test]
    fn status_change_preserves_other_writable_bits() {
        let mut expected = Vec::new();
        // Current status has WPEN set. Setting block protection must keep it.
        expected.extend(rdsr(0b1000_0000));
        expected.extend(wren());
        expected.extend(wrsr(0b1000_1100)); // WPEN preserved plus BlockProtection::All
        expected.extend(rdsr(0x00));

        let mut cat = build(CAT25128, &expected);
        cat.set_block_protection(BlockProtection::All).unwrap();
        finish(cat);
    }

    #[test]
    fn read_rejects_out_of_bounds() {
        let mut cat = build(CAT25128, &[]);
        let mut buf = [0u8; 4];
        assert_eq!(cat.read(0x3FFF, &mut buf), Err(Error::OutOfBounds));
        finish(cat);
    }

    #[test]
    fn write_rejects_out_of_bounds() {
        let mut cat = build(CAT25128, &[]);
        assert_eq!(cat.write(16_383, &[0, 0]), Err(Error::OutOfBounds));
        finish(cat);
    }
}
