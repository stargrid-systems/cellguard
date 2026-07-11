//! The programming layer for AVR Dx targets (NVMCTRL v2).
//!
//! [`Programmer`] unlocks the target, resets it into programming mode, and
//! erases, writes, and reads back flash over the [`Updi`] link.
//!
//! The data-space addresses, NVM command values, key strings, and status bits
//! below are transcribed from the AVR128DB datasheet and cross-checked against
//! `pymcuprog`. They are BENCH-VERIFY: the mock-target tests exercise the
//! sequencing, not the real silicon values.

use crate::link::{RESET_RELEASE, RESET_REQUEST, Updi, UpdiError, UpdiLink, cs};

// --- AVR Dx (NVMCTRL v2) data-space layout. BENCH-VERIFY. ---

/// NVMCTRL command register.
pub const NVMCTRL_CTRLA: u32 = 0x1000;
/// NVMCTRL status register.
pub const NVMCTRL_STATUS: u32 = 0x1002;
/// Base of program flash in the 24-bit UPDI address space.
pub const FLASH_BASE: u32 = 0x80_0000;

/// Flash page size in bytes (AVR128DB). BENCH-VERIFY.
pub const PAGE_SIZE: u32 = 512;

// NVMCTRL.CTRLA commands.
pub const CMD_NONE: u8 = 0x00;
pub const CMD_FLWR: u8 = 0x02;
pub const CMD_FLPER: u8 = 0x08;

// NVMCTRL.STATUS flags.
pub const STATUS_FBUSY: u8 = 1 << 0;
pub const STATUS_ERROR_MASK: u8 = 0x70;

// Unlock keys. Sent least-significant byte first by `Updi::key`.
const KEY_NVMPROG: &[u8; 8] = b"NVMProg ";
const KEY_CHIPERASE: &[u8; 8] = b"NVMErase";

// ASI_KEY_STATUS bits.
pub const KEYSTAT_CHIPERASE: u8 = 1 << 3;
pub const KEYSTAT_NVMPROG: u8 = 1 << 4;

// ASI_SYS_STATUS bits.
pub const SYS_LOCKSTATUS: u8 = 1 << 0;
pub const SYS_NVMPROG: u8 = 1 << 3;

/// Guard time written to UPDI.CTRLA. `0` selects the largest guard time, the
/// safest choice for bring-up. BENCH-VERIFY, tune for speed later.
const GUARD_TIME: u8 = 0x00;

/// Bound on status-poll loops. Each iteration is a wire transaction.
const MAX_POLL: u32 = 100_000;

/// A programming error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProgError<E> {
    /// A link-layer or transport error.
    Updi(UpdiError<E>),
    /// The target did not respond to a status read after a BREAK.
    NotAlive,
    /// The unlock key was not accepted.
    KeyRejected,
    /// The target is locked. A chip erase is required first.
    Locked,
    /// The target never reported programming mode.
    EnterTimeout,
    /// An NVM operation stayed busy past the poll bound.
    Busy,
    /// The NVM controller reported a write error.
    NvmError,
}

impl<E> From<UpdiError<E>> for ProgError<E> {
    fn from(e: UpdiError<E>) -> Self {
        Self::Updi(e)
    }
}

/// A UPDI programmer for an AVR Dx target.
pub struct Programmer<L> {
    updi: Updi<L>,
}

impl<L: UpdiLink> Programmer<L> {
    /// Wraps a transport.
    pub const fn new(link: L) -> Self {
        Self {
            updi: Updi::new(link),
        }
    }

    /// Releases the transport.
    pub fn free(self) -> L {
        self.updi.free()
    }

    /// Enters programming mode: BREAK, confirm the target is alive, set the
    /// guard time, unlock with the NVMPROG key, and reset into programming
    /// mode.
    ///
    /// # Errors
    ///
    /// Returns [`ProgError::NotAlive`], [`ProgError::KeyRejected`],
    /// [`ProgError::Locked`], or [`ProgError::EnterTimeout`] on the matching
    /// failure, or [`ProgError::Updi`] on a transport error.
    pub fn enter(&mut self) -> Result<(), ProgError<L::Error>> {
        self.updi.break_()?;
        if self.updi.ldcs(cs::STATUSA)? == 0 {
            return Err(ProgError::NotAlive);
        }
        self.updi.stcs(cs::CTRLA, GUARD_TIME)?;
        self.updi.key(KEY_NVMPROG)?;
        if self.updi.ldcs(cs::ASI_KEY_STATUS)? & KEYSTAT_NVMPROG == 0 {
            return Err(ProgError::KeyRejected);
        }
        self.reset()?;
        self.wait_prog_mode()
    }

    /// Erases the whole chip with the chip-erase key, clearing the lock. Follow
    /// with [`Programmer::enter`] to program the erased device.
    ///
    /// # Errors
    ///
    /// Returns [`ProgError::NotAlive`] or [`ProgError::KeyRejected`] on the
    /// matching failure, or [`ProgError::Updi`] on a transport error.
    pub fn chip_erase(&mut self) -> Result<(), ProgError<L::Error>> {
        self.updi.break_()?;
        if self.updi.ldcs(cs::STATUSA)? == 0 {
            return Err(ProgError::NotAlive);
        }
        self.updi.key(KEY_CHIPERASE)?;
        if self.updi.ldcs(cs::ASI_KEY_STATUS)? & KEYSTAT_CHIPERASE == 0 {
            return Err(ProgError::KeyRejected);
        }
        self.reset()
    }

    /// Erases the flash page containing `flash_offset`.
    ///
    /// # Errors
    ///
    /// Returns [`ProgError::Busy`] or [`ProgError::NvmError`] on an NVM
    /// failure, or [`ProgError::Updi`] on a transport error.
    pub fn erase_flash_page(&mut self, flash_offset: u32) -> Result<(), ProgError<L::Error>> {
        self.nvm_command(CMD_FLPER)?;
        // On NVMCTRL v2 a write to any address in the page triggers the erase.
        self.updi
            .sts8(FLASH_BASE.saturating_add(flash_offset), 0xFF)?;
        self.wait_flash_ready()?;
        self.nvm_command(CMD_NONE)?;
        Ok(())
    }

    /// Writes `data` to flash at `flash_offset`. The page must be erased first.
    ///
    /// # Errors
    ///
    /// Returns [`ProgError::Busy`] or [`ProgError::NvmError`] on an NVM
    /// failure, or [`ProgError::Updi`] on a transport error.
    pub fn write_flash(
        &mut self,
        flash_offset: u32,
        data: &[u8],
    ) -> Result<(), ProgError<L::Error>> {
        if data.is_empty() {
            return Ok(());
        }
        self.nvm_command(CMD_FLWR)?;
        self.updi
            .set_pointer(FLASH_BASE.saturating_add(flash_offset))?;
        self.updi.st_inc(data)?;
        self.wait_flash_ready()?;
        self.nvm_command(CMD_NONE)?;
        Ok(())
    }

    /// Reads `buf.len()` bytes from flash at `flash_offset`.
    ///
    /// # Errors
    ///
    /// Returns [`ProgError::Updi`] on a transport error.
    pub fn read_flash(
        &mut self,
        flash_offset: u32,
        buf: &mut [u8],
    ) -> Result<(), ProgError<L::Error>> {
        if buf.is_empty() {
            return Ok(());
        }
        self.updi
            .set_pointer(FLASH_BASE.saturating_add(flash_offset))?;
        self.updi.ld_inc(buf)?;
        Ok(())
    }

    /// Leaves programming mode and lets the target run.
    ///
    /// # Errors
    ///
    /// Returns [`ProgError::Updi`] on a transport error.
    pub fn leave(&mut self) -> Result<(), ProgError<L::Error>> {
        self.reset()
    }

    fn reset(&mut self) -> Result<(), ProgError<L::Error>> {
        self.updi.stcs(cs::ASI_RESET_REQ, RESET_REQUEST)?;
        self.updi.stcs(cs::ASI_RESET_REQ, RESET_RELEASE)?;
        Ok(())
    }

    fn wait_prog_mode(&mut self) -> Result<(), ProgError<L::Error>> {
        for _ in 0..MAX_POLL {
            let status = self.updi.ldcs(cs::ASI_SYS_STATUS)?;
            if status & SYS_LOCKSTATUS != 0 {
                return Err(ProgError::Locked);
            }
            if status & SYS_NVMPROG != 0 {
                return Ok(());
            }
        }
        Err(ProgError::EnterTimeout)
    }

    fn nvm_command(&mut self, cmd: u8) -> Result<(), ProgError<L::Error>> {
        self.updi.sts8(NVMCTRL_CTRLA, cmd)?;
        Ok(())
    }

    fn wait_flash_ready(&mut self) -> Result<(), ProgError<L::Error>> {
        for _ in 0..MAX_POLL {
            let status = self.updi.lds8(NVMCTRL_STATUS)?;
            if status & STATUS_ERROR_MASK != 0 {
                return Err(ProgError::NvmError);
            }
            if status & STATUS_FBUSY == 0 {
                return Ok(());
            }
        }
        Err(ProgError::Busy)
    }
}

#[cfg(test)]
mod tests {
    use super::{PAGE_SIZE, ProgError, Programmer};
    use crate::mock::MockTarget;

    fn ramp(n: usize) -> [u8; 600] {
        let mut a = [0u8; 600];
        for (i, b) in a.iter_mut().enumerate().take(n) {
            *b = u8::try_from(i % 251).unwrap();
        }
        a
    }

    #[test]
    fn enter_write_read_roundtrip() {
        let mut prog = Programmer::new(MockTarget::new());
        prog.enter().unwrap();

        let data = ramp(300);
        let payload = &data[..300];
        prog.erase_flash_page(0).unwrap();
        prog.write_flash(0, payload).unwrap();

        let mut back = [0u8; 300];
        prog.read_flash(0, &mut back).unwrap();
        assert_eq!(&back[..], payload);
    }

    #[test]
    fn write_spans_repeat_blocks() {
        // 300 bytes forces two REPEAT blocks (>256), exercising the chunking.
        let mut prog = Programmer::new(MockTarget::new());
        prog.enter().unwrap();
        let data = ramp(300);
        prog.erase_flash_page(0).unwrap();
        prog.write_flash(0, &data[..300]).unwrap();
        let mut back = [0u8; 300];
        prog.read_flash(0, &mut back).unwrap();
        assert_eq!(&back[..], &data[..300]);
    }

    #[test]
    fn erase_sets_page_to_ff() {
        let mut prog = Programmer::new(MockTarget::new());
        prog.enter().unwrap();
        prog.erase_flash_page(0).unwrap();
        prog.write_flash(0, &[0x11, 0x22, 0x33]).unwrap();
        // Erasing again restores 0xFF across the page.
        prog.erase_flash_page(0).unwrap();
        let mut back = [0u8; 3];
        prog.read_flash(0, &mut back).unwrap();
        assert_eq!(back, [0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn locked_target_cannot_enter() {
        let mut prog = Programmer::new(MockTarget::locked());
        assert_eq!(prog.enter(), Err(ProgError::Locked));
    }

    #[test]
    fn chip_erase_unlocks_then_enter_succeeds() {
        let mut prog = Programmer::new(MockTarget::locked());
        prog.chip_erase().unwrap();
        prog.enter().unwrap();
        // Programmable after recovery.
        prog.erase_flash_page(0).unwrap();
        prog.write_flash(0, &[0xAB, 0xCD]).unwrap();
        let mut back = [0u8; 2];
        prog.read_flash(0, &mut back).unwrap();
        assert_eq!(back, [0xAB, 0xCD]);
    }

    #[test]
    fn second_page_is_addressed() {
        let mut prog = Programmer::new(MockTarget::new());
        prog.enter().unwrap();
        let off = PAGE_SIZE;
        prog.erase_flash_page(off).unwrap();
        prog.write_flash(off, &[0x5A; 4]).unwrap();
        let mut back = [0u8; 4];
        prog.read_flash(off, &mut back).unwrap();
        assert_eq!(back, [0x5A; 4]);
    }
}
