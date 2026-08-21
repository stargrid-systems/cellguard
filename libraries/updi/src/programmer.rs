//! The programming layer for AVR Dx targets (NVMCTRL v2).
//!
//! [`Programmer`] unlocks the target, resets it into programming mode, and
//! erases, writes, and reads back flash over the [`Updi`] driver.
//!
//! The data-space addresses, NVM command values, key strings, and status bits
//! below come from the AVR128DB datasheet, cross-checked against `pymcuprog`.

use crate::driver::{REPEAT_MAX, RESET_RELEASE, RESET_REQUEST, Updi, UpdiError, cs};
use crate::link::UpdiLink;

/// Base of program flash in the 24-bit UPDI address space.
pub const FLASH_BASE: u32 = 0x80_0000;
/// Total program-flash size in bytes (AVR128DB).
pub const FLASH_SIZE: u32 = 128 * 1024;
/// Flash page size in bytes (AVR128DB).
pub const PAGE_SIZE: u32 = 512;

/// NVM controller registers, commands, and status flags.
pub mod nvmctrl {
    /// Command register.
    pub const CTRLA: u32 = 0x1000;
    /// Status register.
    pub const STATUS: u32 = 0x1002;

    /// No command (disarm the controller).
    pub const CMD_NONE: u8 = 0x00;
    /// Flash write.
    pub const CMD_FLWR: u8 = 0x02;
    /// Flash page erase.
    pub const CMD_FLPER: u8 = 0x08;

    /// Flash busy.
    pub const STATUS_FBUSY: u8 = 1 << 0;
    /// Any write-error bit.
    pub const STATUS_ERROR_MASK: u8 = 0x70;
}

/// ASI status-register bits, read over the CS space.
pub mod asi {
    /// Chip-erase key accepted (`ASI_KEY_STATUS`).
    pub const KEYSTAT_CHIPERASE: u8 = 1 << 3;
    /// NVMPROG key accepted (`ASI_KEY_STATUS`).
    pub const KEYSTAT_NVMPROG: u8 = 1 << 4;
    /// Target locked (`ASI_SYS_STATUS`).
    pub const SYS_LOCKSTATUS: u8 = 1 << 0;
    /// Programming mode active (`ASI_SYS_STATUS`).
    pub const SYS_NVMPROG: u8 = 1 << 3;
}

/// Unlock keys. Sent least-significant byte first by `Updi::key`.
const KEY_NVMPROG: &[u8; 8] = b"NVMProg ";
const KEY_CHIPERASE: &[u8; 8] = b"NVMErase";

/// Guard time written to UPDI.CTRLA. `0` selects the largest guard time, the
/// safest choice for bring-up. A shorter guard time is faster.
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
    /// A chip erase did not complete within the poll bound.
    EraseTimeout,
    /// A flash offset was misaligned or out of range.
    InvalidOffset,
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
        if self.updi.ldcs(cs::ASI_KEY_STATUS)? & asi::KEYSTAT_NVMPROG == 0 {
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
    /// Returns [`ProgError::NotAlive`], [`ProgError::KeyRejected`], or
    /// [`ProgError::EraseTimeout`] on the matching failure, or
    /// [`ProgError::Updi`] on a transport error.
    pub fn chip_erase(&mut self) -> Result<(), ProgError<L::Error>> {
        self.updi.break_()?;
        if self.updi.ldcs(cs::STATUSA)? == 0 {
            return Err(ProgError::NotAlive);
        }
        self.updi.key(KEY_CHIPERASE)?;
        if self.updi.ldcs(cs::ASI_KEY_STATUS)? & asi::KEYSTAT_CHIPERASE == 0 {
            return Err(ProgError::KeyRejected);
        }
        self.reset()?;
        self.wait_erase_done()
    }

    /// Erases the flash page starting at `flash_offset`.
    ///
    /// # Errors
    ///
    /// Returns [`ProgError::InvalidOffset`] if `flash_offset` is not
    /// page-aligned or is out of range, [`ProgError::Busy`] or
    /// [`ProgError::NvmError`] on an NVM failure, or [`ProgError::Updi`] on
    /// a transport error.
    pub fn erase_flash_page(&mut self, flash_offset: u32) -> Result<(), ProgError<L::Error>> {
        if !flash_offset.is_multiple_of(PAGE_SIZE) || flash_offset >= FLASH_SIZE {
            return Err(ProgError::InvalidOffset);
        }
        self.nvm_command(nvmctrl::CMD_FLPER)?;
        // On NVMCTRL v2 a write to any address in the page triggers the erase.
        let r = self.do_erase(flash_offset);
        // Always disarm, even on failure, so a later read cannot misfire into
        // the still-armed controller.
        let disarm = self.nvm_command(nvmctrl::CMD_NONE);
        r.and(disarm)
    }

    fn do_erase(&mut self, flash_offset: u32) -> Result<(), ProgError<L::Error>> {
        self.updi.sts8(FLASH_BASE + flash_offset, 0xFF)?;
        self.wait_flash_ready()
    }

    /// Writes `data` to flash at `flash_offset`. Every page it touches must be
    /// erased first.
    ///
    /// `flash_offset` must be word-aligned (even). Data that spans a page
    /// boundary is programmed one page at a time.
    ///
    /// # Errors
    ///
    /// Returns [`ProgError::InvalidOffset`] if `flash_offset` is misaligned or
    /// the write runs past the end of flash, [`ProgError::Busy`] or
    /// [`ProgError::NvmError`] on an NVM failure, or [`ProgError::Updi`] on a
    /// transport error.
    pub fn write_flash(
        &mut self,
        flash_offset: u32,
        data: &[u8],
    ) -> Result<(), ProgError<L::Error>> {
        if data.is_empty() {
            return Ok(());
        }
        // Flash programs in 16-bit words, so the start must be word-aligned.
        if !flash_offset.is_multiple_of(2) {
            return Err(ProgError::InvalidOffset);
        }
        let len = u32::try_from(data.len()).map_err(|_| ProgError::InvalidOffset)?;
        let end = flash_offset
            .checked_add(len)
            .ok_or(ProgError::InvalidOffset)?;
        if end > FLASH_SIZE {
            return Err(ProgError::InvalidOffset);
        }

        let mut offset = flash_offset;
        let mut rest = data;
        while !rest.is_empty() {
            // Bytes up to the next page boundary belong to the current page.
            let page_end = (offset / PAGE_SIZE + 1) * PAGE_SIZE;
            let to_boundary = usize::try_from(page_end - offset).unwrap_or(rest.len());
            if rest.len() <= to_boundary {
                self.write_page_segment(offset, rest)?;
                break;
            }
            let (segment, tail) = rest.split_at(to_boundary);
            self.write_page_segment(offset, segment)?;
            offset = page_end;
            rest = tail;
        }
        Ok(())
    }

    fn write_page_segment(
        &mut self,
        offset: u32,
        segment: &[u8],
    ) -> Result<(), ProgError<L::Error>> {
        self.nvm_command(nvmctrl::CMD_FLWR)?;
        let r = self.stream_words(offset, segment);
        // Always disarm, even on failure.
        let disarm = self.nvm_command(nvmctrl::CMD_NONE);
        r.and(disarm)
    }

    fn stream_words(&mut self, offset: u32, segment: &[u8]) -> Result<(), ProgError<L::Error>> {
        self.updi.set_pointer(FLASH_BASE + offset)?;
        // Split off an odd trailing byte so the even head streams as whole words.
        let (head, tail) = if segment.len().is_multiple_of(2) {
            (segment, &[][..])
        } else {
            segment.split_at(segment.len() - 1)
        };
        // AVR Dx pages (512 B) exceed one REPEAT block, so stream in blocks.
        for block in head.chunks(REPEAT_MAX) {
            self.updi.st_inc(block)?;
        }
        if let [last] = tail {
            // Pad the odd tail with the erased value so the controller commits
            // the final word instead of leaving it half written.
            self.updi.st_inc(&[*last, 0xFF])?;
        }
        self.wait_flash_ready()
    }

    /// Reads `buf.len()` bytes from flash at `flash_offset`.
    ///
    /// # Errors
    ///
    /// Returns [`ProgError::InvalidOffset`] if the read runs past the end of
    /// flash, or [`ProgError::Updi`] on a transport error.
    pub fn read_flash(
        &mut self,
        flash_offset: u32,
        buf: &mut [u8],
    ) -> Result<(), ProgError<L::Error>> {
        if buf.is_empty() {
            return Ok(());
        }
        let len = u32::try_from(buf.len()).map_err(|_| ProgError::InvalidOffset)?;
        let end = flash_offset
            .checked_add(len)
            .ok_or(ProgError::InvalidOffset)?;
        if end > FLASH_SIZE {
            return Err(ProgError::InvalidOffset);
        }
        self.updi.set_pointer(FLASH_BASE + flash_offset)?;
        // AVR Dx reads can exceed one REPEAT block, so read in blocks.
        for block in buf.chunks_mut(REPEAT_MAX) {
            self.updi.ld_inc(block)?;
        }
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
            if status & asi::SYS_LOCKSTATUS != 0 {
                return Err(ProgError::Locked);
            }
            if status & asi::SYS_NVMPROG != 0 {
                return Ok(());
            }
        }
        Err(ProgError::EnterTimeout)
    }

    fn wait_erase_done(&mut self) -> Result<(), ProgError<L::Error>> {
        // The chip erase clears the lock. Wait for it so a following enter()
        // does not race a target that is still erasing. A target that was
        // already unlocked reads LOCKSTATUS clear from the first poll, so also
        // wait for the NVM controller to leave FBUSY. Data-space reads work
        // once unlocked, which is exactly when this check runs.
        for _ in 0..MAX_POLL {
            if self.updi.ldcs(cs::ASI_SYS_STATUS)? & asi::SYS_LOCKSTATUS == 0 {
                return self.wait_flash_ready();
            }
        }
        Err(ProgError::EraseTimeout)
    }

    fn nvm_command(&mut self, cmd: u8) -> Result<(), ProgError<L::Error>> {
        self.updi.sts8(nvmctrl::CTRLA, cmd)?;
        Ok(())
    }

    fn wait_flash_ready(&mut self) -> Result<(), ProgError<L::Error>> {
        for _ in 0..MAX_POLL {
            let status = self.updi.lds8(nvmctrl::STATUS)?;
            if status & nvmctrl::STATUS_ERROR_MASK != 0 {
                return Err(ProgError::NvmError);
            }
            if status & nvmctrl::STATUS_FBUSY == 0 {
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
        prog.write_flash(0, &[0x11, 0x22, 0x33, 0x44]).unwrap();
        // Erasing again restores 0xFF across the page.
        prog.erase_flash_page(0).unwrap();
        let mut back = [0u8; 4];
        prog.read_flash(0, &mut back).unwrap();
        assert_eq!(back, [0xFF, 0xFF, 0xFF, 0xFF]);
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

    #[test]
    fn write_across_page_boundary_roundtrips() {
        let mut prog = Programmer::new(MockTarget::new());
        prog.enter().unwrap();
        // Straddle the 512-byte boundary: both touched pages must be erased.
        prog.erase_flash_page(0).unwrap();
        prog.erase_flash_page(PAGE_SIZE).unwrap();
        let data = ramp(40);
        let off = PAGE_SIZE - 20;
        prog.write_flash(off, &data[..40]).unwrap();
        let mut back = [0u8; 40];
        prog.read_flash(off, &mut back).unwrap();
        assert_eq!(&back[..], &data[..40]);
    }

    #[test]
    fn odd_length_write_roundtrips() {
        let mut prog = Programmer::new(MockTarget::new());
        prog.enter().unwrap();
        prog.erase_flash_page(0).unwrap();
        let data = [0x01, 0x02, 0x03];
        prog.write_flash(0, &data).unwrap();
        let mut back = [0u8; 3];
        prog.read_flash(0, &mut back).unwrap();
        assert_eq!(back, data);
    }

    #[test]
    fn misaligned_offsets_are_rejected() {
        let mut prog = Programmer::new(MockTarget::new());
        prog.enter().unwrap();
        // Odd write start.
        assert_eq!(
            prog.write_flash(1, &[0xAA, 0xBB]),
            Err(ProgError::InvalidOffset)
        );
        // Non-page-aligned erase.
        assert_eq!(prog.erase_flash_page(1), Err(ProgError::InvalidOffset));
        // Out-of-range erase.
        assert_eq!(
            prog.erase_flash_page(super::FLASH_SIZE),
            Err(ProgError::InvalidOffset)
        );
    }

    #[test]
    fn write_disarms_controller_on_nvm_error() {
        // The controller must be disarmed after a failed write so a following
        // read does not land in an armed write command.
        let mut prog = Programmer::new(MockTarget::failing());
        prog.enter().unwrap();
        prog.erase_flash_page(0).unwrap();
        assert_eq!(prog.write_flash(0, &[0x11, 0x22]), Err(ProgError::NvmError));
        // The command register was reset to CMD_NONE despite the error.
        assert_eq!(prog.free().nvm_command(), super::nvmctrl::CMD_NONE);
    }
}
