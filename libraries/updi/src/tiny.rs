//! The programming layer for tinyAVR 0/1-series targets.
//!
//! [`TinyProgrammer`] unlocks the target, resets it into programming mode, and
//! erases, writes, and reads back flash over the [`Updi`] driver using 16-bit
//! data-space addresses.
//!
//! The NVMCTRL command values and status bits come from the `ATtiny406` PAC
//! (`avr_device::attiny406::nvmctrl`) and the ATtiny406/416 datasheets.
//!
//! On NVMCTRL P0 (tinyAVR 0/1-series) writing a command to `CTRLA` **executes**
//! it immediately against the address or page buffer already loaded. This is
//! the opposite of AVR Dx (NVMCTRL v2), where `CTRLA` **arms** a command and a
//! subsequent data write triggers it. The erase and write flows below follow
//! the execute model: set up the address/data first, then write the command.

use crate::driver::{RESET_RELEASE, RESET_REQUEST, Updi, cs};
use crate::link::UpdiLink;
pub use crate::programmer::ProgError;

/// NVMCTRL base in the 16-bit data space (shared by tinyAVR 0/1-series).
const NVMCTRL_BASE: u16 = 0x1000;

/// Base of program flash in the 16-bit data space.
pub const FLASH_BASE: u16 = 0x8000;

/// Total flash size for ATtiny406/ATtiny416 (4 KB). tinyAVR data space is
/// 16-bit, so the addressing type is `u16`.
pub const FLASH_SIZE: u16 = 4096;

/// Flash page size in bytes.
pub const PAGE_SIZE: u16 = 16;

/// NVM controller registers, commands, and status flags.
pub mod nvmctrl {
    /// Command register (CTRLA).
    pub const CTRLA: u16 = super::NVMCTRL_BASE;
    /// Status register.
    pub const STATUS: u16 = super::NVMCTRL_BASE + 0x02;

    /// Write page.
    pub const CMD_WP: u8 = 0x01;
    /// Page erase.
    pub const CMD_ER: u8 = 0x02;
    /// Page buffer clear.
    pub const CMD_PBC: u8 = 0x04;

    /// Flash busy (STATUS bit 0).
    pub const STATUS_FBUSY: u8 = 1 << 0;
    /// Write error (STATUS bit 2).
    pub const STATUS_WRERROR: u8 = 1 << 2;
}

/// ASI status-register bits (shared by all UPDI devices).
pub mod asi {
    /// Chip-erase key accepted.
    pub const KEYSTAT_CHIPERASE: u8 = 1 << 3;
    /// NVMPROG key accepted.
    pub const KEYSTAT_NVMPROG: u8 = 1 << 4;
    /// Target locked.
    pub const SYS_LOCKSTATUS: u8 = 1 << 0;
    /// Programming mode active.
    pub const SYS_NVMPROG: u8 = 1 << 3;
}

const KEY_NVMPROG: &[u8; 8] = b"NVMProg ";
const KEY_CHIPERASE: &[u8; 8] = b"NVMErase";
const GUARD_TIME: u8 = 0x00;
/// Poll bound for NVM and mode transitions, in round-trips. At 115200 baud
/// one LDCS/LDS poll is a few hundred microseconds, so 255 polls budgets
/// roughly 100 ms per wait: an order of magnitude beyond real NVM timings
/// (page writes are single-digit milliseconds, chip erase tens), and far
/// inside the firmware watchdog that bounds a whole command anyway. A u16
/// countdown would cost two registers and compare per iteration.
const MAX_POLL: u8 = u8::MAX;

/// A UPDI programmer for a tinyAVR 0/1-series target.
pub struct TinyProgrammer<L> {
    updi: Updi<L>,
}

impl<L: UpdiLink> TinyProgrammer<L> {
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

    /// Enters programming mode.
    ///
    /// # Errors
    ///
    /// See [`ProgError`].
    pub fn enter(&mut self) -> Result<(), ProgError<L::Error>> {
        self.begin(KEY_NVMPROG, asi::KEYSTAT_NVMPROG, true)
    }

    /// Erases the whole chip with the chip-erase key, clearing the lock. Follow
    /// with [`Self::enter`] to program the erased device.
    ///
    /// # Errors
    ///
    /// See [`ProgError`].
    pub fn chip_erase(&mut self) -> Result<(), ProgError<L::Error>> {
        let done = self.begin(KEY_CHIPERASE, asi::KEYSTAT_CHIPERASE, false);
        done.and_then(|()| self.wait_flash_ready())
    }

    /// Runs a key cycle in one sequence: reset the UPDI state machine, check
    /// the target is alive, set the guard time, send `key`, confirm it was
    /// accepted, reset the target, then wait for the key's effect
    /// (`SYS_NVMPROG` set when `prog_mode`, else `SYS_LOCKSTATUS` clear).
    ///
    /// One fused body serves both keys. Splitting handshake and wait across
    /// call sites costs a call, a prologue, and a `Result` handoff more per
    /// key path, measurably.
    ///
    /// Both paths set the guard time. `GUARD_TIME` is the CTRLA reset value,
    /// so the extra write on the chip-erase path is a no-op that keeps one
    /// code sequence serving both keys.
    #[expect(
        clippy::trivially_copy_pass_by_ref,
        reason = "a by-value key would stack-copy 8 bytes at each call site"
    )]
    fn begin(
        &mut self,
        key: &[u8; 8],
        accepted: u8,
        prog_mode: bool,
    ) -> Result<(), ProgError<L::Error>> {
        self.updi.break_()?;
        if self.updi.ldcs(cs::STATUSA)? == 0 {
            return Err(ProgError::NotAlive);
        }
        self.updi.stcs(cs::CTRLA, GUARD_TIME)?;
        self.updi.key(key)?;
        if self.updi.ldcs(cs::ASI_KEY_STATUS)? & accepted == 0 {
            return Err(ProgError::KeyRejected);
        }
        self.reset()?;
        for _ in 0..MAX_POLL {
            let status = self.updi.ldcs(cs::ASI_SYS_STATUS)?;
            let lock = status & asi::SYS_LOCKSTATUS;
            if prog_mode {
                if lock != 0 {
                    return Err(ProgError::Locked);
                }
                if status & asi::SYS_NVMPROG != 0 {
                    return Ok(());
                }
            } else if lock == 0 {
                return Ok(());
            }
        }
        Err(if prog_mode {
            ProgError::EnterTimeout
        } else {
            ProgError::EraseTimeout
        })
    }

    /// Erases the flash page starting at `flash_offset`.
    ///
    /// On NVMCTRL P0 the page address must be loaded before the erase command
    /// executes. A dummy store to any address in the page sets the target, then
    /// `CMD_ER` erases it.
    ///
    /// # Errors
    ///
    /// See [`ProgError`].
    pub fn erase_flash_page(&mut self, flash_offset: u16) -> Result<(), ProgError<L::Error>> {
        if !flash_offset.is_multiple_of(PAGE_SIZE) || flash_offset >= FLASH_SIZE {
            return Err(ProgError::InvalidOffset);
        }
        self.wait_flash_ready()?;
        self.updi
            .sts8_16(FLASH_BASE.wrapping_add(flash_offset), 0xFF)?;
        self.nvm_command(nvmctrl::CMD_ER)?;
        self.wait_flash_ready()
    }

    /// Writes `data` to flash at `flash_offset`.
    ///
    /// Each page touched must be erased first. Data that spans a page boundary
    /// is programmed one page at a time.
    ///
    /// tinyAVR data space is 16-bit, so the offset is `u16`. Flash is 4 KiB
    /// and always fits.
    ///
    /// # Errors
    ///
    /// See [`ProgError`].
    pub fn write_flash(
        &mut self,
        flash_offset: u16,
        data: &[u8],
    ) -> Result<(), ProgError<L::Error>> {
        if data.is_empty() {
            return Ok(());
        }
        if !flash_offset.is_multiple_of(2) {
            return Err(ProgError::InvalidOffset);
        }
        // Both flash_offset and the length are bounded by FLASH_SIZE, so the
        // end cannot wrap u16.
        let end = flash_offset
            .checked_add(u16::try_from(data.len()).unwrap_or(u16::MAX))
            .ok_or(ProgError::InvalidOffset)?;
        if end > FLASH_SIZE {
            return Err(ProgError::InvalidOffset);
        }

        let mut offset = flash_offset;
        let mut rest = data;
        while !rest.is_empty() {
            let page_end = (offset | (PAGE_SIZE - 1)) + 1;
            let take = rest.len().min(usize::from(page_end - offset));
            let (segment, tail) = rest.split_at(take);
            self.write_page(offset, segment)?;
            offset += u16::try_from(take).unwrap_or(0);
            rest = tail;
        }
        Ok(())
    }

    /// Clears the page buffer, loads it with `segment`, then commits with
    /// `CMD_WP`. On P0 the buffer must be loaded before the write command
    /// executes.
    fn write_page(&mut self, offset: u16, segment: &[u8]) -> Result<(), ProgError<L::Error>> {
        self.wait_flash_ready()?;
        self.commit(nvmctrl::CMD_PBC)?;
        self.updi.set_pointer_16(FLASH_BASE.wrapping_add(offset))?;
        self.updi.st_inc(segment)?;
        self.commit(nvmctrl::CMD_WP)
    }

    /// Executes an NVMCTRL command and waits for the controller to settle.
    /// Every P0 command sequence ends this way, so one fused helper replaces
    /// the command/wait call pair at each site.
    fn commit(&mut self, cmd: u8) -> Result<(), ProgError<L::Error>> {
        self.nvm_command(cmd)?;
        self.wait_flash_ready()
    }

    /// Reads `buf.len()` bytes from flash at `flash_offset`.
    ///
    /// # Errors
    ///
    /// See [`ProgError`].
    pub fn read_flash(
        &mut self,
        flash_offset: u16,
        buf: &mut [u8],
    ) -> Result<(), ProgError<L::Error>> {
        if buf.is_empty() {
            return Ok(());
        }
        let end = flash_offset
            .checked_add(u16::try_from(buf.len()).unwrap_or(u16::MAX))
            .ok_or(ProgError::InvalidOffset)?;
        if end > FLASH_SIZE {
            return Err(ProgError::InvalidOffset);
        }
        self.updi
            .set_pointer_16(FLASH_BASE.wrapping_add(flash_offset))?;
        self.updi.ld_inc(buf)?;
        Ok(())
    }

    /// Leaves programming mode and lets the target run.
    ///
    /// # Errors
    ///
    /// See [`ProgError`].
    pub fn leave(&mut self) -> Result<(), ProgError<L::Error>> {
        self.reset()
    }

    fn reset(&mut self) -> Result<(), ProgError<L::Error>> {
        self.updi.stcs(cs::ASI_RESET_REQ, RESET_REQUEST)?;
        self.updi.stcs(cs::ASI_RESET_REQ, RESET_RELEASE)?;
        Ok(())
    }

    fn nvm_command(&mut self, cmd: u8) -> Result<(), ProgError<L::Error>> {
        self.updi.sts8_16(nvmctrl::CTRLA, cmd)?;
        Ok(())
    }

    fn wait_flash_ready(&mut self) -> Result<(), ProgError<L::Error>> {
        for _ in 0..MAX_POLL {
            let status = self.updi.lds8_16(nvmctrl::STATUS)?;
            if status & nvmctrl::STATUS_WRERROR != 0 {
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
    use super::{FLASH_SIZE, PAGE_SIZE, ProgError, TinyProgrammer};
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
        let mut prog = TinyProgrammer::new(MockTarget::tiny());
        prog.enter().unwrap();

        let data = ramp(64);
        prog.erase_flash_page(0).unwrap();
        prog.write_flash(0, &data[..64]).unwrap();

        let mut back = [0u8; 64];
        prog.read_flash(0, &mut back).unwrap();
        assert_eq!(&back[..], &data[..64]);
    }

    #[test]
    fn erase_sets_page_to_ff() {
        let mut prog = TinyProgrammer::new(MockTarget::tiny());
        prog.enter().unwrap();
        prog.erase_flash_page(0).unwrap();
        prog.write_flash(0, &[0x11, 0x22, 0x33, 0x44]).unwrap();
        prog.erase_flash_page(0).unwrap();
        let mut back = [0u8; 4];
        prog.read_flash(0, &mut back).unwrap();
        assert_eq!(back, [0xFF, 0xFF, 0xFF, 0xFF]);
    }

    #[test]
    fn locked_target_cannot_enter() {
        let mut prog = TinyProgrammer::new(MockTarget::tiny_locked());
        assert_eq!(prog.enter(), Err(ProgError::Locked));
    }

    #[test]
    fn chip_erase_unlocks_then_enter_succeeds() {
        let mut prog = TinyProgrammer::new(MockTarget::tiny_locked());
        prog.chip_erase().unwrap();
        prog.enter().unwrap();
        prog.erase_flash_page(0).unwrap();
        prog.write_flash(0, &[0xAB, 0xCD]).unwrap();
        let mut back = [0u8; 2];
        prog.read_flash(0, &mut back).unwrap();
        assert_eq!(back, [0xAB, 0xCD]);
    }

    #[test]
    fn second_page_is_addressed() {
        let mut prog = TinyProgrammer::new(MockTarget::tiny());
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
        let mut prog = TinyProgrammer::new(MockTarget::tiny());
        prog.enter().unwrap();
        prog.erase_flash_page(0).unwrap();
        prog.erase_flash_page(PAGE_SIZE).unwrap();
        let data = ramp(20);
        let off = PAGE_SIZE - 8;
        prog.write_flash(off, &data[..20]).unwrap();
        let mut back = [0u8; 20];
        prog.read_flash(off, &mut back).unwrap();
        assert_eq!(&back[..], &data[..20]);
    }

    #[test]
    fn odd_length_write_roundtrips() {
        let mut prog = TinyProgrammer::new(MockTarget::tiny());
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
        let mut prog = TinyProgrammer::new(MockTarget::tiny());
        prog.enter().unwrap();
        assert_eq!(
            prog.write_flash(1, &[0xAA, 0xBB]),
            Err(ProgError::InvalidOffset)
        );
        assert_eq!(prog.erase_flash_page(1), Err(ProgError::InvalidOffset));
        assert_eq!(
            prog.erase_flash_page(FLASH_SIZE),
            Err(ProgError::InvalidOffset)
        );
    }

    #[test]
    fn write_reports_nvm_error() {
        let mut prog = TinyProgrammer::new(MockTarget::tiny_failing());
        prog.enter().unwrap();
        prog.erase_flash_page(0).unwrap();
        assert_eq!(prog.write_flash(0, &[0x11, 0x22]), Err(ProgError::NvmError));
    }
}
