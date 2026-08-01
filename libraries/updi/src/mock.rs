//! A mock UPDI target for host tests.
//!
//! [`MockTarget`] implements [`UpdiLink`] by emulating the
//! target side of the protocol: it parses the instruction byte stream, keeps
//! the CS and ASI registers, an NVM controller, and a flash array, and returns
//! the bytes a real slave would. It lets the link and programmer layers run
//! end to end without hardware. It emulates only the instructions this crate
//! emits.
//!
//! The mock supports two NVM controller models. [`MockTarget::new`] emulates
//! NVMCTRL v2 (AVR Dx): writing a command to CTRLA *arms* it, and a subsequent
//! data write *triggers* the armed operation. [`MockTarget::tiny`] emulates
//! NVMCTRL P0 (tinyAVR 0/1-series): writing a command to CTRLA *executes* it
//! against the address or page buffer already loaded, so data must come first.

use crate::driver::{
    cs, ACK, ADDR_16, ADDR_24, OP_KEY, OP_LD, OP_LDCS, OP_LDS, OP_REPEAT, OP_ST, OP_STCS, OP_STS,
    PTR_INC, PTR_SET, RESET_RELEASE, RESET_REQUEST, SIZE_16, SIZE_24, SYNCH,
};
use crate::link::UpdiLink;
use crate::programmer::asi;

/// Mask for the opcode field (the high three bits of an instruction byte).
const OP_MASK: u8 = 0xE0;
const FLASH_LEN: usize = 1024;
const RESP_CAP: usize = 512;
/// Page-buffer capacity (large enough for the biggest page, AVR Dx 512 B).
const PAGE_BUF_SIZE: usize = 512;

/// Which NVM controller model the mock emulates.
#[derive(Clone, Copy, PartialEq)]
enum NvmModel {
    /// AVR Dx (NVMCTRL v2): arm-then-trigger.
    AvrDx,
    /// tinyAVR 0/1-series (NVMCTRL P0): execute-on-write.
    TinyAvr,
}

/// Which store a run of address bytes belongs to.
#[derive(Clone, Copy)]
enum AddrKind {
    Load,
    Store,
    SetPointer,
}

enum Parse {
    Synch,
    Opcode,
    Stcs(u8),
    Addr {
        kind: AddrKind,
        need: u8,
        got: u8,
        acc: u32,
    },
    StoreValue(u32),
    StoreInc(usize),
    Repeat,
    Key {
        got: u8,
        buf: [u8; 8],
    },
}

/// An emulated UPDI target: CS/ASI registers, an NVM controller, and a small
/// flash array. Enough to run the link and programmer layers end to end.
pub struct MockTarget {
    model: NvmModel,
    flash: [u8; FLASH_LEN],
    /// Page buffer for the P0 model. Unused on v2.
    page_buf: [u8; PAGE_BUF_SIZE],
    /// Flash-array index of the page the page buffer / erase targets. P0 only.
    target_page: usize,
    /// Flash base in data space (`0x80_0000` for v2, `0x8000` for P0).
    flash_base: u32,
    /// Flash page size in bytes.
    page_size: usize,
    key_status: u8,
    sys_status: u8,
    locked: bool,
    reset_pending: bool,
    fail_nvm: bool,
    nvm_status: u8,
    nvm_cmd: u8,
    pointer: u32,
    repeat: usize,
    state: Parse,
    resp: [u8; RESP_CAP],
    resp_len: usize,
    resp_head: usize,
}

impl MockTarget {
    /// A fresh, unlocked AVR Dx target with erased flash.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            model: NvmModel::AvrDx,
            flash: [0xFF; FLASH_LEN],
            page_buf: [0xFF; PAGE_BUF_SIZE],
            target_page: 0,
            flash_base: crate::programmer::FLASH_BASE,
            page_size: crate::programmer::PAGE_SIZE as usize,
            key_status: 0,
            sys_status: 0,
            locked: false,
            reset_pending: false,
            fail_nvm: false,
            nvm_status: 0,
            nvm_cmd: 0,
            pointer: 0,
            repeat: 1,
            state: Parse::Synch,
            resp: [0u8; RESP_CAP],
            resp_len: 0,
            resp_head: 0,
        }
    }

    /// A locked AVR Dx target: only a chip erase clears the lock.
    #[must_use]
    pub const fn locked() -> Self {
        let mut t = Self::new();
        t.locked = true;
        t
    }

    /// An AVR Dx target whose NVM controller reports an error on every flash
    /// write.
    #[must_use]
    pub const fn failing() -> Self {
        let mut t = Self::new();
        t.fail_nvm = true;
        t
    }

    /// A fresh, unlocked tinyAVR target (NVMCTRL P0) with erased flash.
    #[must_use]
    pub const fn tiny() -> Self {
        let mut t = Self::new();
        t.model = NvmModel::TinyAvr;
        t.flash_base = crate::tiny::FLASH_BASE as u32;
        t.page_size = crate::tiny::PAGE_SIZE as usize;
        t
    }

    /// A locked tinyAVR target.
    #[must_use]
    pub const fn tiny_locked() -> Self {
        let mut t = Self::tiny();
        t.locked = true;
        t
    }

    /// A tinyAVR target whose NVM controller reports a write error on every
    /// page commit.
    #[must_use]
    pub const fn tiny_failing() -> Self {
        let mut t = Self::tiny();
        t.fail_nvm = true;
        t
    }

    /// Reads a flash byte, for test assertions.
    #[must_use]
    pub fn flash_at(&self, offset: usize) -> u8 {
        self.flash.get(offset).copied().unwrap_or(0)
    }

    /// The current NVM command register value, for test assertions.
    #[must_use]
    pub const fn nvm_command(&self) -> u8 {
        self.nvm_cmd
    }

    fn push(&mut self, byte: u8) {
        if let Some(slot) = self.resp.get_mut(self.resp_len) {
            *slot = byte;
            self.resp_len += 1;
        }
    }

    const fn take_repeat(&mut self) -> usize {
        let n = self.repeat;
        self.repeat = 1;
        n
    }

    fn flash_index(&self, addr: u32) -> Option<usize> {
        addr.checked_sub(self.flash_base).and_then(|delta| {
            let idx = delta as usize;
            if idx < FLASH_LEN {
                Some(idx)
            } else {
                None
            }
        })
    }

    fn ctrl_addr(&self) -> u32 {
        match self.model {
            NvmModel::AvrDx => crate::programmer::nvmctrl::CTRLA,
            NvmModel::TinyAvr => u32::from(crate::tiny::nvmctrl::CTRLA),
        }
    }

    fn status_addr(&self) -> u32 {
        match self.model {
            NvmModel::AvrDx => crate::programmer::nvmctrl::STATUS,
            NvmModel::TinyAvr => u32::from(crate::tiny::nvmctrl::STATUS),
        }
    }

    fn data_read(&self, addr: u32) -> u8 {
        if addr == self.status_addr() {
            return self.nvm_status;
        }
        self.flash_index(addr)
            .map_or(0, |i| self.flash.get(i).copied().unwrap_or(0))
    }

    fn data_write(&mut self, addr: u32, val: u8) {
        match self.model {
            NvmModel::AvrDx => self.data_write_v2(addr, val),
            NvmModel::TinyAvr => self.data_write_p0(addr, val),
        }
    }

    /// AVR Dx (NVMCTRL v2): writing CTRLA arms a command; a subsequent data
    /// write to a flash address triggers the armed operation.
    fn data_write_v2(&mut self, addr: u32, val: u8) {
        if addr == self.ctrl_addr() {
            self.nvm_cmd = val;
            self.nvm_status = 0;
            return;
        }
        let Some(idx) = self.flash_index(addr) else {
            return;
        };
        let page_size = self.page_size;
        match self.nvm_cmd {
            crate::programmer::nvmctrl::CMD_FLPER => {
                let page = (idx / page_size) * page_size;
                for cell in self.flash.iter_mut().skip(page).take(page_size) {
                    *cell = 0xFF;
                }
            }
            crate::programmer::nvmctrl::CMD_FLWR => {
                if self.fail_nvm {
                    self.nvm_status = crate::programmer::nvmctrl::STATUS_ERROR_MASK;
                }
                if let Some(slot) = self.flash.get_mut(idx) {
                    *slot = val;
                }
            }
            _ => {}
        }
    }

    /// tinyAVR (NVMCTRL P0): writing CTRLA executes the command against the
    /// page buffer or target page already loaded. Flash data writes load the
    /// page buffer and set the target page.
    fn data_write_p0(&mut self, addr: u32, val: u8) {
        if addr == self.ctrl_addr() {
            self.nvm_cmd = val;
            self.nvm_status = 0;
            let page_size = self.page_size;
            match val {
                crate::tiny::nvmctrl::CMD_ER => {
                    for cell in self.flash.iter_mut().skip(self.target_page).take(page_size) {
                        *cell = 0xFF;
                    }
                }
                crate::tiny::nvmctrl::CMD_WP => {
                    if self.fail_nvm {
                        self.nvm_status = crate::tiny::nvmctrl::STATUS_WRERROR;
                    }
                    let dst = self.flash.iter_mut().skip(self.target_page);
                    for (i, cell) in dst.take(page_size).enumerate() {
                        *cell = self.page_buf.get(i).copied().unwrap_or(0xFF);
                    }
                }
                crate::tiny::nvmctrl::CMD_PBC => {
                    for slot in &mut self.page_buf {
                        *slot = 0xFF;
                    }
                }
                _ => {}
            }
            return;
        }
        let Some(idx) = self.flash_index(addr) else {
            return;
        };
        let page_size = self.page_size;
        self.target_page = idx - (idx % page_size);
        if let Some(slot) = self.page_buf.get_mut(idx % page_size) {
            *slot = val;
        }
    }

    const fn cs_read(&self, reg: u8) -> u8 {
        match reg {
            cs::STATUSA => 0x30,
            cs::ASI_KEY_STATUS => self.key_status,
            cs::ASI_SYS_STATUS => {
                self.sys_status | if self.locked { asi::SYS_LOCKSTATUS } else { 0 }
            }
            _ => 0,
        }
    }

    const fn cs_write(&mut self, reg: u8, val: u8) {
        if reg != cs::ASI_RESET_REQ {
            return;
        }
        if val == RESET_REQUEST {
            self.reset_pending = true;
        } else if val == RESET_RELEASE && self.reset_pending {
            self.apply_reset();
            self.reset_pending = false;
        }
    }

    const fn apply_reset(&mut self) {
        if self.key_status & asi::KEYSTAT_CHIPERASE != 0 {
            self.flash = [0xFF; FLASH_LEN];
            self.page_buf = [0xFF; PAGE_BUF_SIZE];
            self.locked = false;
            self.key_status = 0;
            self.sys_status = 0;
            self.nvm_status = 0;
        } else if self.key_status & asi::KEYSTAT_NVMPROG != 0 {
            self.sys_status |= asi::SYS_NVMPROG;
        }
    }

    fn process_key(&mut self, sent: [u8; 8]) {
        let mut key = sent;
        key.reverse();
        if &key == b"NVMProg " {
            self.key_status |= asi::KEYSTAT_NVMPROG;
        } else if &key == b"NVMErase" {
            self.key_status |= asi::KEYSTAT_CHIPERASE;
        }
    }

    fn feed(&mut self, byte: u8) {
        self.state = match core::mem::replace(&mut self.state, Parse::Synch) {
            Parse::Synch => {
                if byte == SYNCH {
                    Parse::Opcode
                } else {
                    Parse::Synch
                }
            }
            Parse::Opcode => self.opcode(byte),
            Parse::Stcs(reg) => {
                self.cs_write(reg, byte);
                Parse::Synch
            }
            Parse::Addr {
                kind,
                need,
                got,
                acc,
            } => {
                let acc = acc | (u32::from(byte) << (8 * u32::from(got)));
                let got = got + 1;
                if got < need {
                    Parse::Addr {
                        kind,
                        need,
                        got,
                        acc,
                    }
                } else {
                    self.finish_addr(kind, acc)
                }
            }
            Parse::StoreValue(addr) => {
                self.data_write(addr, byte);
                self.push(ACK);
                Parse::Synch
            }
            Parse::StoreInc(rem) => {
                let addr = self.pointer;
                self.data_write(addr, byte);
                self.pointer = self.pointer.wrapping_add(1);
                self.push(ACK);
                if rem <= 1 {
                    Parse::Synch
                } else {
                    Parse::StoreInc(rem - 1)
                }
            }
            Parse::Repeat => {
                self.repeat = usize::from(byte) + 1;
                Parse::Synch
            }
            Parse::Key { got, mut buf } => {
                if let Some(slot) = buf.get_mut(got as usize) {
                    *slot = byte;
                }
                let got = got + 1;
                if got < 8 {
                    Parse::Key { got, buf }
                } else {
                    self.process_key(buf);
                    Parse::Synch
                }
            }
        };
    }

    /// Extracts the address byte count from an LDS/STS opcode (bits 3:2).
    const fn lds_sts_addr_len(op: u8) -> u8 {
        match op & 0x0C {
            ADDR_16 => 2,
            ADDR_24 => 3,
            _ => 0,
        }
    }

    /// Extracts the address byte count from a pointer-set ST opcode (bits 1:0).
    const fn ptr_set_addr_len(op: u8) -> u8 {
        match op & 0x03 {
            SIZE_16 => 2,
            SIZE_24 => 3,
            _ => 0,
        }
    }

    fn opcode(&mut self, op: u8) -> Parse {
        match op & OP_MASK {
            OP_LDCS => {
                let v = self.cs_read(op & 0x0F);
                self.push(v);
                Parse::Synch
            }
            OP_STCS => Parse::Stcs(op & 0x0F),
            OP_LDS => Parse::Addr {
                kind: AddrKind::Load,
                need: Self::lds_sts_addr_len(op),
                got: 0,
                acc: 0,
            },
            OP_STS => Parse::Addr {
                kind: AddrKind::Store,
                need: Self::lds_sts_addr_len(op),
                got: 0,
                acc: 0,
            },
            OP_ST if op & 0x0C == PTR_SET => Parse::Addr {
                kind: AddrKind::SetPointer,
                need: Self::ptr_set_addr_len(op),
                got: 0,
                acc: 0,
            },
            OP_ST if op & 0x0C == PTR_INC => Parse::StoreInc(self.take_repeat()),
            OP_LD if op & 0x0C == PTR_INC => {
                for _ in 0..self.take_repeat() {
                    let v = self.data_read(self.pointer);
                    self.pointer = self.pointer.wrapping_add(1);
                    self.push(v);
                }
                Parse::Synch
            }
            OP_REPEAT => Parse::Repeat,
            OP_KEY => Parse::Key {
                got: 0,
                buf: [0u8; 8],
            },
            _ => Parse::Synch,
        }
    }

    fn finish_addr(&mut self, kind: AddrKind, addr: u32) -> Parse {
        match kind {
            AddrKind::Load => {
                let v = self.data_read(addr);
                self.push(v);
                Parse::Synch
            }
            AddrKind::Store => {
                self.push(ACK);
                Parse::StoreValue(addr)
            }
            AddrKind::SetPointer => {
                self.pointer = addr;
                self.push(ACK);
                Parse::Synch
            }
        }
    }
}

impl Default for MockTarget {
    fn default() -> Self {
        Self::new()
    }
}

impl UpdiLink for MockTarget {
    type Error = ();

    fn break_(&mut self) -> Result<(), ()> {
        self.state = Parse::Synch;
        self.resp_len = 0;
        self.resp_head = 0;
        self.reset_pending = false;
        Ok(())
    }

    fn send(&mut self, data: &[u8]) -> Result<(), ()> {
        for &b in data {
            self.feed(b);
        }
        Ok(())
    }

    fn recv(&mut self, buf: &mut [u8]) -> Result<(), ()> {
        for b in buf.iter_mut() {
            if self.resp_head >= self.resp_len {
                return Err(());
            }
            *b = self.resp.get(self.resp_head).copied().ok_or(())?;
            self.resp_head += 1;
        }
        if self.resp_head == self.resp_len {
            self.resp_head = 0;
            self.resp_len = 0;
        }
        Ok(())
    }
}
