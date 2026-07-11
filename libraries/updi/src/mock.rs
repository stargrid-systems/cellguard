//! A mock UPDI target for host tests.
//!
//! [`MockTarget`] implements [`UpdiLink`](crate::UpdiLink) by emulating the
//! target side of the protocol: it parses the instruction byte stream, keeps
//! the CS and ASI registers, an NVM controller, and a flash array, and returns
//! the bytes a real slave would. It lets the link and programmer layers run
//! end to end without hardware. It emulates only the instructions this crate
//! emits.

use crate::driver::{
    ACK, OP_KEY, OP_LD, OP_LDCS, OP_LDS, OP_REPEAT, OP_ST, OP_STCS, OP_STS, PTR_INC, PTR_SET,
    RESET_RELEASE, RESET_REQUEST, SYNCH, cs,
};
use crate::link::UpdiLink;
use crate::programmer::{FLASH_BASE, PAGE_SIZE, asi, nvmctrl};

/// Mask for the opcode field (the high three bits of an instruction byte).
const OP_MASK: u8 = 0xE0;
const FLASH_LEN: usize = 1024;
const RESP_CAP: usize = 512;

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

/// An emulated AVR Dx UPDI target: CS/ASI registers, an NVM controller, and a
/// small flash array. Enough to run the link and programmer layers end to end.
pub struct MockTarget {
    flash: [u8; FLASH_LEN],
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
    /// A fresh, unlocked target with erased flash.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            flash: [0xFF; FLASH_LEN],
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

    /// A locked target: only a chip erase clears the lock.
    #[must_use]
    pub const fn locked() -> Self {
        let mut t = Self::new();
        t.locked = true;
        t
    }

    /// A target whose NVM controller reports an error on every flash write.
    #[must_use]
    pub const fn failing() -> Self {
        let mut t = Self::new();
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

    const fn flash_index(addr: u32) -> Option<usize> {
        match addr.checked_sub(FLASH_BASE) {
            Some(delta) => {
                let idx = delta as usize;
                if idx < FLASH_LEN { Some(idx) } else { None }
            }
            None => None,
        }
    }

    fn data_read(&self, addr: u32) -> u8 {
        if addr == nvmctrl::STATUS {
            // The STATUS register is latched at write time and never busy here.
            // A failing target keeps reporting the error until the next command
            // is armed. This does not depend on which command is armed at read
            // time, so the mock stays honest if the programmer reorders its
            // disarm.
            return self.nvm_status;
        }
        Self::flash_index(addr).map_or(0, |i| self.flash.get(i).copied().unwrap_or(0))
    }

    fn data_write(&mut self, addr: u32, val: u8) {
        if addr == nvmctrl::CTRLA {
            self.nvm_cmd = val;
            // Arming a new command clears a previously latched write error.
            self.nvm_status = 0;
            return;
        }
        let Some(idx) = Self::flash_index(addr) else {
            return;
        };
        match self.nvm_cmd {
            nvmctrl::CMD_FLPER => {
                let page = (idx / PAGE_SIZE as usize) * PAGE_SIZE as usize;
                for cell in self.flash.iter_mut().skip(page).take(PAGE_SIZE as usize) {
                    *cell = 0xFF;
                }
            }
            nvmctrl::CMD_FLWR => {
                // A failing target latches the error but still commits the byte,
                // matching a controller that flags the fault after the write.
                if self.fail_nvm {
                    self.nvm_status = nvmctrl::STATUS_ERROR_MASK;
                }
                if let Some(slot) = self.flash.get_mut(idx) {
                    *slot = val;
                }
            }
            _ => {}
        }
    }

    const fn cs_read(&self, reg: u8) -> u8 {
        match reg {
            cs::STATUSA => 0x30, // nonzero: alive, UPDI revision in the high nibble
            cs::ASI_KEY_STATUS => self.key_status,
            cs::ASI_SYS_STATUS => {
                self.sys_status | if self.locked { asi::SYS_LOCKSTATUS } else { 0 }
            }
            _ => 0,
        }
    }

    const fn cs_write(&mut self, reg: u8, val: u8) {
        if reg != cs::ASI_RESET_REQ {
            return; // CTRLA guard time and others: ignore
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
            self.locked = false;
            self.key_status = 0;
            self.sys_status = 0;
            self.nvm_status = 0;
        } else if self.key_status & asi::KEYSTAT_NVMPROG != 0 {
            // Enters programming mode, but a locked device stays locked.
            self.sys_status |= asi::SYS_NVMPROG;
        }
    }

    fn process_key(&mut self, sent: [u8; 8]) {
        // Keys travel least-significant byte first, so reverse to recover them.
        let mut key = [0u8; 8];
        for (dst, src) in key.iter_mut().zip(sent.iter().rev()) {
            *dst = *src;
        }
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
                need: 3,
                got: 0,
                acc: 0,
            },
            OP_STS => Parse::Addr {
                kind: AddrKind::Store,
                need: 3,
                got: 0,
                acc: 0,
            },
            OP_ST if op & 0x0C == PTR_SET => Parse::Addr {
                kind: AddrKind::SetPointer,
                need: 3,
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
        // A BREAK resets the comms state machine, not the system state.
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
                return Err(()); // underflow: the stack read more than was produced
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
