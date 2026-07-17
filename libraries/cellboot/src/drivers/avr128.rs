//! On-chip NVM adapters for AVR128, backed by `avrxt-hal`.
//!
//! Both adapters borrow a single shared [`Nvm`] and a [`CcpUnlock`] handle
//! (the device `CPU`), because there is one `NVMCTRL` peripheral but several
//! logical stores: the state store in EEPROM, the key store in USERROW, and the
//! flash writer for self-programming. The firmware owns the `Nvm` and hands
//! each adapter a reference.

use avrxt_hal::clock::CcpUnlock;
use avrxt_hal::nvmctrl::{FlashInstance, Nvm, NvmError, NvmInstance};

use crate::io::{KeyStore, NvmWriter, PagedFlash, StateStore, write_with_page_erase};

/// A [`StateStore`] backed by a slot in the on-chip EEPROM.
///
/// The slot starts at `offset` and is `len` bytes long. Loads and stores are
/// rejected if they exceed the slot, so the updater's state cannot spill into a
/// neighbouring region.
pub struct EepromState<'a, T: NvmInstance, C: CcpUnlock> {
    nvm: &'a Nvm<T>,
    cpu: &'a C,
    offset: u16,
    len: u16,
}

impl<'a, T: NvmInstance, C: CcpUnlock> EepromState<'a, T, C> {
    /// Binds a state store to the EEPROM slot `[offset, offset + len)`.
    #[must_use]
    pub const fn new(nvm: &'a Nvm<T>, cpu: &'a C, offset: u16, len: u16) -> Self {
        Self {
            nvm,
            cpu,
            offset,
            len,
        }
    }

    const fn fits(&self, wanted: usize) -> Result<(), NvmError> {
        if wanted > self.len as usize {
            Err(NvmError::OutOfBounds)
        } else {
            Ok(())
        }
    }
}

impl<T: NvmInstance, C: CcpUnlock> StateStore for EepromState<'_, T, C> {
    type Error = NvmError;

    fn load(&mut self, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.fits(buf.len())?;
        self.nvm.read_eeprom(self.offset, buf)
    }

    fn store(&mut self, data: &[u8]) -> Result<(), Self::Error> {
        self.fits(data.len())?;
        self.nvm.write_eeprom(self.offset, data, self.cpu)
    }
}

/// A development-only [`KeyStore`] backed by the AVR128 USERROW.
///
/// Production locks the USERROW and uses [`NoKeyStore`](crate::io::NoKeyStore)
/// instead; this writable path exists so a key can be replaced over a trusted
/// bus during bring-up. Writing the key erases the whole USERROW page, so the
/// key must own it (it is written from offset 0).
pub struct UserRowKeyStore<'a, T: FlashInstance, C: CcpUnlock> {
    nvm: &'a Nvm<T>,
    cpu: &'a C,
}

impl<'a, T: FlashInstance, C: CcpUnlock> UserRowKeyStore<'a, T, C> {
    /// Binds a key store to the USERROW.
    #[must_use]
    pub const fn new(nvm: &'a Nvm<T>, cpu: &'a C) -> Self {
        Self { nvm, cpu }
    }
}

impl<T: FlashInstance, C: CcpUnlock> KeyStore for UserRowKeyStore<'_, T, C> {
    type Error = NvmError;

    fn write_key(&mut self, key: &[u8]) -> Result<(), Self::Error> {
        self.nvm.write_userrow(key, self.cpu)
    }
}

/// An [`NvmWriter`] backed by `Nvm` flash self-programming.
///
/// This adapts [`Nvm::write_flash`] to the streaming [`NvmWriter`] contract.
/// Each flash page is erased the first time a write touches it, then the bytes
/// stream straight to flash, so a sub-page or page-straddling chunk is handled
/// without buffering a whole page. The page-erase bookkeeping mirrors the UPDI
/// programmer: a single `erased_page` tracker assumes writes arrive in
/// ascending, contiguous order.
pub struct FlashNvmWriter<'a, T: FlashInstance, C: CcpUnlock> {
    nvm: &'a Nvm<T>,
    cpu: &'a C,
    erased_page: Option<u32>,
}

impl<'a, T: FlashInstance, C: CcpUnlock> FlashNvmWriter<'a, T, C> {
    /// Binds a flash writer to a shared `Nvm` and `CPU` handle.
    #[must_use]
    pub const fn new(nvm: &'a Nvm<T>, cpu: &'a C) -> Self {
        Self {
            nvm,
            cpu,
            erased_page: None,
        }
    }
}

impl<T: FlashInstance, C: CcpUnlock> NvmWriter for FlashNvmWriter<'_, T, C> {
    type Error = NvmError;

    fn begin(&mut self) -> Result<(), Self::Error> {
        self.erased_page = None;
        Ok(())
    }

    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error> {
        // `self.nvm`/`self.cpu` (via the adapter) and `self.erased_page` are
        // disjoint fields, so both can be borrowed mutably in the same call.
        let mut adapter = NvmAdapter { nvm: self.nvm, cpu: self.cpu };
        write_with_page_erase(
            address,
            data,
            T::FLASH_PAGE_SIZE,
            &mut self.erased_page,
            &mut adapter,
        )
    }

    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.nvm.read_flash(self.cpu, address, buf)
    }

    fn finish(&mut self) -> Result<(), Self::Error> {
        // Self-programming has no programming-mode handshake to leave: flash
        // is live the moment a write commits, so there is nothing to do here
        // but report success. The trait's `leave` step is meaningful only for
        // external programmers (UPDI) that halt the target.
        Ok(())
    }
}

/// Local adapter exposing `Nvm` page-erase and write through the
/// `PagedFlash` seam that `write_with_page_erase` drives.
struct NvmAdapter<'a, T: FlashInstance, C: CcpUnlock> {
    nvm: &'a Nvm<T>,
    cpu: &'a C,
}

impl<T: FlashInstance, C: CcpUnlock> PagedFlash for NvmAdapter<'_, T, C> {
    type Error = NvmError;

    fn erase_page(&mut self, page_base: u32) -> Result<(), Self::Error> {
        self.nvm.erase_flash_page(self.cpu, page_base)
    }

    fn write_chunk(&mut self, addr: u32, chunk: &[u8]) -> Result<(), Self::Error> {
        self.nvm.write_flash(self.cpu, addr, chunk)
    }
}
