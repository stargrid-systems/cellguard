//! On-chip NVM adapters for AVR128, backed by `avrxt-hal`.
//!
//! Both adapters borrow a single shared [`Nvm`] and a [`CcpUnlock`] handle
//! (the device `CPU`), because there is one `NVMCTRL` peripheral but two
//! logical stores: the state store in EEPROM and the key store in USERROW. The
//! firmware owns the `Nvm` and hands each adapter a reference.

use avrxt_hal::clock::CcpUnlock;
use avrxt_hal::nvmctrl::{Nvm, NvmError, NvmInstance};

use crate::io::{KeyStore, StateStore};

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
        Self { nvm, cpu, offset, len }
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
pub struct UserRowKeyStore<'a, T: NvmInstance, C: CcpUnlock> {
    nvm: &'a Nvm<T>,
    cpu: &'a C,
}

impl<'a, T: NvmInstance, C: CcpUnlock> UserRowKeyStore<'a, T, C> {
    /// Binds a key store to the USERROW.
    #[must_use]
    pub const fn new(nvm: &'a Nvm<T>, cpu: &'a C) -> Self {
        Self { nvm, cpu }
    }
}

impl<T: NvmInstance, C: CcpUnlock> KeyStore for UserRowKeyStore<'_, T, C> {
    type Error = NvmError;

    fn write_key(&mut self, key: &[u8]) -> Result<(), Self::Error> {
        self.nvm.write_userrow(key, self.cpu)
    }
}
