//! The programming layer for tinyAVR 0/1-series targets.
//!
//! [`TinyProgrammer`] unlocks the target, resets it into programming mode, and
//! erases, writes, and reads back flash over the [`Updi`] driver using 16-bit
//! data-space addresses.
//!
//! The NVMCTRL command values and status bits come from the `ATtiny406` PAC
//! (`avr_device::attiny406::nvmctrl`) and the ATtiny406/416 datasheets.

use crate::driver::{RESET_RELEASE, RESET_REQUEST, Updi, cs};
use crate::link::UpdiLink;
pub use crate::programmer::ProgError;

/// NVMCTRL base in the 16-bit data space (shared by tinyAVR 0/1-series).
const NVMCTRL_BASE: u16 = 0x1000;

/// Base of program flash in the 16-bit data space.
pub const FLASH_BASE: u16 = 0x8000;

/// Total flash size for ATtiny406/ATtiny416 (4 KB).
pub const FLASH_SIZE: u32 = 4096;

/// Flash page size in bytes.
pub const PAGE_SIZE: u32 = 16;

/// NVM controller registers, commands, and status flags.
pub mod nvmctrl {
    /// Command register (CTRLA).
    pub const CTRLA: u16 = super::NVMCTRL_BASE;
    /// Status register.
    pub const STATUS: u16 = super::NVMCTRL_BASE + 0x02;

    /// No command (disarm).
    pub const CMD_NONE: u8 = 0x00;
    /// Write page.
    pub const CMD_WP: u8 = 0x01;
    /// Page erase.
    pub const CMD_ER: u8 = 0x02;
    /// Erase and write page.
    pub const CMD_ERWP: u8 = 0x03;
    /// Page buffer clear.
    pub const CMD_PBC: u8 = 0x04;
    /// Chip erase.
    pub const CMD_CHER: u8 = 0x05;

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
const MAX_POLL: u32 = 100_000;

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

    /// Erases the whole chip with the chip-erase key.
    ///
    /// # Errors
    ///
    /// See [`ProgError`].
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
    /// See [`ProgError`].
    pub fn erase_flash_page(&mut self, flash_offset: u32) -> Result<(), ProgError<L::Error>> {
        if !flash_offset.is_multiple_of(PAGE_SIZE) || flash_offset >= FLASH_SIZE {
            return Err(ProgError::InvalidOffset);
        }
        self.nvm_command(nvmctrl::CMD_ER)?;
        let r = self.do_erase(flash_offset);
        let disarm = self.nvm_command(nvmctrl::CMD_NONE);
        r.and(disarm)
    }

    fn do_erase(&mut self, flash_offset: u32) -> Result<(), ProgError<L::Error>> {
        let addr =
            FLASH_BASE + u16::try_from(flash_offset).map_err(|_| ProgError::InvalidOffset)?;
        self.updi.sts8_16(addr, 0xFF)?;
        self.wait_flash_ready()
    }

    /// Writes `data` to flash at `flash_offset`.
    ///
    /// Each page touched must be erased first. The final partial page is padded
    /// with `0xFF` so the page buffer commits.
    ///
    /// # Errors
    ///
    /// See [`ProgError`].
    pub fn write_flash(
        &mut self,
        flash_offset: u32,
        data: &[u8],
    ) -> Result<(), ProgError<L::Error>> {
        if data.is_empty() {
            return Ok(());
        }
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
            let page_end = (offset / PAGE_SIZE + 1) * PAGE_SIZE;
            let to_boundary = usize::try_from(page_end - offset).unwrap_or(rest.len());
            if rest.len() <= to_boundary {
                self.write_page(offset, rest)?;
                break;
            }
            let (segment, tail) = rest.split_at(to_boundary);
            self.write_page(offset, segment)?;
            offset = page_end;
            rest = tail;
        }
        Ok(())
    }

    fn write_page(&mut self, offset: u32, segment: &[u8]) -> Result<(), ProgError<L::Error>> {
        self.nvm_command(nvmctrl::CMD_WP)?;
        let r = self.stream_page(offset, segment);
        let disarm = self.nvm_command(nvmctrl::CMD_NONE);
        r.and(disarm)
    }

    fn stream_page(&mut self, offset: u32, segment: &[u8]) -> Result<(), ProgError<L::Error>> {
        let addr = FLASH_BASE + u16::try_from(offset).map_err(|_| ProgError::InvalidOffset)?;
        self.updi.set_pointer_16(addr)?;

        let page_remain = usize::try_from(PAGE_SIZE - (offset % PAGE_SIZE)).unwrap_or(0);
        if segment.len() < page_remain {
            // Partial page: pad with 0xFF so the tinyAVR page buffer commits.
            let mut padded = [0xFFu8; PAGE_SIZE as usize];
            let (dst, _) = padded.split_at_mut(segment.len());
            dst.copy_from_slice(segment);
            let (data, _) = padded.split_at(page_remain);
            self.updi.st_inc(data)?;
        } else {
            self.updi.st_inc(segment)?;
        }
        self.wait_flash_ready()
    }

    /// Reads `buf.len()` bytes from flash at `flash_offset`.
    ///
    /// # Errors
    ///
    /// See [`ProgError`].
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
        let addr =
            FLASH_BASE + u16::try_from(flash_offset).map_err(|_| ProgError::InvalidOffset)?;
        self.updi.set_pointer_16(addr)?;
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
        for _ in 0..MAX_POLL {
            if self.updi.ldcs(cs::ASI_SYS_STATUS)? & asi::SYS_LOCKSTATUS == 0 {
                return self.wait_flash_ready();
            }
        }
        Err(ProgError::EraseTimeout)
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
