//! The UPDI driver: the instruction set over a [`UpdiLink`] transport.
//!
//! Every instruction is prefixed with the SYNCH byte `0x55` so the target can
//! measure the host baud. Stores are two-phase with an ACK after the address
//! and after the data. `REPEAT` blocks a load or store so a page streams under
//! one opcode.

use crate::link::UpdiLink;

/// The SYNCH character that prefixes every instruction.
pub const SYNCH: u8 = 0x55;
/// The ACK byte a target returns after accepting a store phase.
pub const ACK: u8 = 0x40;

// Instruction opcodes (high three bits).
pub const OP_LDS: u8 = 0x00;
pub const OP_LD: u8 = 0x20;
pub const OP_STS: u8 = 0x40;
pub const OP_ST: u8 = 0x60;
pub const OP_LDCS: u8 = 0x80;
pub const OP_REPEAT: u8 = 0xA0;
pub const OP_STCS: u8 = 0xC0;
pub const OP_KEY: u8 = 0xE0;

// LDS/STS address-size field (bits 3:2). Data size is one byte (0) throughout.
pub const ADDR_24: u8 = 2 << 2;
pub const ADDR_16: u8 = 1 << 2;
// LD/ST pointer control (bits 3:2).
pub const PTR_INC: u8 = 1 << 2;
pub const PTR_SET: u8 = 2 << 2;
// Pointer-set size field (bits 1:0).
pub const SIZE_24: u8 = 2;
pub const SIZE_16: u8 = 1;
/// Largest block one `REPEAT` can cover (count is a single byte, so 256).
const REPEAT_MAX: usize = 256;

/// UPDI control- and status-register addresses (the `CS` space).
pub mod cs {
    /// UPDI status register A.
    pub const STATUSA: u8 = 0x00;
    /// UPDI control register A (guard time).
    pub const CTRLA: u8 = 0x02;
    /// ASI key status. Reports which unlock key was accepted.
    pub const ASI_KEY_STATUS: u8 = 0x07;
    /// ASI reset request.
    pub const ASI_RESET_REQ: u8 = 0x08;
    /// ASI system status. Reports lock state and programming mode.
    pub const ASI_SYS_STATUS: u8 = 0x0B;
}

/// Value written to `ASI_RESET_REQ` to request a reset.
pub const RESET_REQUEST: u8 = 0x59;
/// Value written to `ASI_RESET_REQ` to release the reset.
pub const RESET_RELEASE: u8 = 0x00;

/// A link-layer error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum UpdiError<E> {
    /// The transport failed.
    Link(E),
    /// A store phase was not acknowledged.
    NoAck,
}

impl<E> From<E> for UpdiError<E> {
    fn from(e: E) -> Self {
        Self::Link(e)
    }
}

/// The UPDI link layer over a [`UpdiLink`] transport.
pub struct Updi<L> {
    link: L,
}

impl<L: UpdiLink> Updi<L> {
    /// Wraps a transport.
    pub const fn new(link: L) -> Self {
        Self { link }
    }

    /// Releases the transport.
    pub fn free(self) -> L {
        self.link
    }

    /// Sends a BREAK to reset the target's UPDI state machine.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::Link`] if the transport fails.
    pub fn break_(&mut self) -> Result<(), UpdiError<L::Error>> {
        self.link.break_()?;
        Ok(())
    }

    /// Reads a control/status register.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::Link`] if the transport fails.
    pub fn ldcs(&mut self, reg: u8) -> Result<u8, UpdiError<L::Error>> {
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_LDCS | (reg & 0x0F))?;
        self.link.recv_byte().map_err(UpdiError::Link)
    }

    /// Writes a control/status register. `STCS` is not acknowledged.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::Link`] if the transport fails.
    pub fn stcs(&mut self, reg: u8, val: u8) -> Result<(), UpdiError<L::Error>> {
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_STCS | (reg & 0x0F))?;
        self.link.send_byte(val).map_err(UpdiError::Link)
    }

    /// Loads one byte from a 24-bit data-space address.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::Link`] if the transport fails.
    pub fn lds8(&mut self, addr: u32) -> Result<u8, UpdiError<L::Error>> {
        let [a0, a1, a2, _] = addr.to_le_bytes();
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_LDS | ADDR_24)?;
        self.link.send_byte(a0)?;
        self.link.send_byte(a1)?;
        self.link.send_byte(a2)?;
        self.link.recv_byte().map_err(UpdiError::Link)
    }

    /// Stores one byte to a 24-bit data-space address.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::NoAck`] if the target does not acknowledge, or
    /// [`UpdiError::Link`] if the transport fails.
    pub fn sts8(&mut self, addr: u32, val: u8) -> Result<(), UpdiError<L::Error>> {
        let [a0, a1, a2, _] = addr.to_le_bytes();
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_STS | ADDR_24)?;
        self.link.send_byte(a0)?;
        self.link.send_byte(a1)?;
        self.link.send_byte(a2)?;
        self.expect_ack()?;
        self.link.send_byte(val)?;
        self.expect_ack()
    }

    /// Sets the pointer register to a 24-bit address for `ld_inc`/`st_inc`.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::NoAck`] if the target does not acknowledge, or
    /// [`UpdiError::Link`] if the transport fails.
    pub fn set_pointer(&mut self, addr: u32) -> Result<(), UpdiError<L::Error>> {
        let [a0, a1, a2, _] = addr.to_le_bytes();
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_ST | PTR_SET | SIZE_24)?;
        self.link.send_byte(a0)?;
        self.link.send_byte(a1)?;
        self.link.send_byte(a2)?;
        self.expect_ack()
    }

    /// Loads one byte from a 16-bit data-space address.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::Link`] if the transport fails.
    pub fn lds8_16(&mut self, addr: u16) -> Result<u8, UpdiError<L::Error>> {
        let [a0, a1] = addr.to_le_bytes();
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_LDS | ADDR_16)?;
        self.link.send_byte(a0)?;
        self.link.send_byte(a1)?;
        self.link.recv_byte().map_err(UpdiError::Link)
    }

    /// Stores one byte to a 16-bit data-space address.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::NoAck`] if the target does not acknowledge, or
    /// [`UpdiError::Link`] if the transport fails.
    pub fn sts8_16(&mut self, addr: u16, val: u8) -> Result<(), UpdiError<L::Error>> {
        let [a0, a1] = addr.to_le_bytes();
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_STS | ADDR_16)?;
        self.link.send_byte(a0)?;
        self.link.send_byte(a1)?;
        self.expect_ack()?;
        self.link.send_byte(val)?;
        self.expect_ack()
    }

    /// Sets the pointer register to a 16-bit address for `ld_inc`/`st_inc`.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::NoAck`] if the target does not acknowledge, or
    /// [`UpdiError::Link`] if the transport fails.
    pub fn set_pointer_16(&mut self, addr: u16) -> Result<(), UpdiError<L::Error>> {
        let [a0, a1] = addr.to_le_bytes();
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_ST | PTR_SET | SIZE_16)?;
        self.link.send_byte(a0)?;
        self.link.send_byte(a1)?;
        self.expect_ack()
    }

    /// Reads `buf.len()` bytes from the pointer, post-incrementing.
    ///
    /// A `REPEAT` blocks the load so all bytes arrive under one opcode. `buf`
    /// must not be empty.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::Link`] if the transport fails.
    pub fn ld_inc(&mut self, buf: &mut [u8]) -> Result<(), UpdiError<L::Error>> {
        for chunk in buf.chunks_mut(REPEAT_MAX) {
            self.repeat(chunk.len())?;
            self.link.send_byte(SYNCH)?;
            self.link.send_byte(OP_LD | PTR_INC)?;
            self.link.recv(chunk)?;
        }
        Ok(())
    }

    /// Writes `data` to the pointer, post-incrementing, acknowledged per byte.
    ///
    /// A `REPEAT` blocks the store so the block streams under one opcode.
    /// `data` must not be empty.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::NoAck`] if a byte is not acknowledged, or
    /// [`UpdiError::Link`] if the transport fails.
    pub fn st_inc(&mut self, data: &[u8]) -> Result<(), UpdiError<L::Error>> {
        for chunk in data.chunks(REPEAT_MAX) {
            self.repeat(chunk.len())?;
            self.link.send_byte(SYNCH)?;
            self.link.send_byte(OP_ST | PTR_INC)?;
            for &byte in chunk {
                self.link.send_byte(byte)?;
                self.expect_ack()?;
            }
        }
        Ok(())
    }

    /// Sends an 8-byte unlock key. Keys travel least-significant byte first.
    ///
    /// # Errors
    ///
    /// Returns [`UpdiError::Link`] if the transport fails.
    pub fn key(&mut self, key: &[u8; 8]) -> Result<(), UpdiError<L::Error>> {
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_KEY)?;
        for &byte in key.iter().rev() {
            self.link.send_byte(byte)?;
        }
        Ok(())
    }

    /// Emits a `REPEAT` so the next instruction runs `count` times. `count`
    /// must be at least one.
    fn repeat(&mut self, count: usize) -> Result<(), UpdiError<L::Error>> {
        // REPEAT takes count-1: the next instruction runs (count-1)+1 times. A
        // single byte bounds a page-sized block to 256, which is enough here.
        let rpt = u8::try_from(count.saturating_sub(1)).unwrap_or(u8::MAX);
        self.link.send_byte(SYNCH)?;
        self.link.send_byte(OP_REPEAT)?;
        self.link.send_byte(rpt).map_err(UpdiError::Link)
    }

    fn expect_ack(&mut self) -> Result<(), UpdiError<L::Error>> {
        if self.link.recv_byte()? == ACK {
            Ok(())
        } else {
            Err(UpdiError::NoAck)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{ACK, Updi, UpdiError, cs};
    use crate::link::UpdiLink;

    /// A fixed-capacity byte buffer, enough for the short frames these tests
    /// use.
    struct Buf {
        data: [u8; 64],
        len: usize,
    }

    impl Buf {
        fn new() -> Self {
            Self {
                data: [0u8; 64],
                len: 0,
            }
        }
        fn from(s: &[u8]) -> Self {
            let mut b = Self::new();
            b.extend(s);
            b
        }
        fn extend(&mut self, s: &[u8]) {
            for &x in s {
                if let Some(slot) = self.data.get_mut(self.len) {
                    *slot = x;
                    self.len += 1;
                }
            }
        }
        fn as_slice(&self) -> &[u8] {
            self.data.get(..self.len).unwrap_or(&[])
        }
    }

    /// A transport that records sent bytes and replays a queue of recv bytes.
    struct RecordLink {
        sent: Buf,
        recv_queue: Buf,
        recv_pos: usize,
    }

    impl RecordLink {
        fn new(recv: &[u8]) -> Self {
            Self {
                sent: Buf::new(),
                recv_queue: Buf::from(recv),
                recv_pos: 0,
            }
        }
    }

    impl UpdiLink for RecordLink {
        type Error = ();

        fn break_(&mut self) -> Result<(), ()> {
            Ok(())
        }

        fn send_byte(&mut self, byte: u8) -> Result<(), ()> {
            self.sent.extend(&[byte]);
            Ok(())
        }

        fn recv_byte(&mut self) -> Result<u8, ()> {
            let byte = *self.recv_queue.as_slice().get(self.recv_pos).ok_or(())?;
            self.recv_pos += 1;
            Ok(byte)
        }
    }

    #[test]
    fn ldcs_sends_synch_and_opcode() {
        let mut updi = Updi::new(RecordLink::new(&[0xAB]));
        assert_eq!(updi.ldcs(cs::STATUSA).unwrap(), 0xAB);
        assert_eq!(updi.link.sent.as_slice(), &[0x55, 0x80]);
    }

    #[test]
    fn stcs_sends_value_no_ack() {
        let mut updi = Updi::new(RecordLink::new(&[]));
        updi.stcs(cs::CTRLA, 0x06).unwrap();
        assert_eq!(updi.link.sent.as_slice(), &[0x55, 0xC0 | 0x02, 0x06]);
    }

    #[test]
    fn sts8_two_phase_with_acks() {
        let mut updi = Updi::new(RecordLink::new(&[ACK, ACK]));
        updi.sts8(0x00_1000, 0x08).unwrap();
        assert_eq!(
            updi.link.sent.as_slice(),
            &[0x55, 0x48, 0x00, 0x10, 0x00, 0x08]
        );
    }

    #[test]
    fn sts8_without_ack_errors() {
        let mut updi = Updi::new(RecordLink::new(&[0x00]));
        assert_eq!(updi.sts8(0x1000, 1), Err(UpdiError::NoAck));
    }

    #[test]
    fn key_is_reversed() {
        let mut updi = Updi::new(RecordLink::new(&[]));
        updi.key(b"NVMProg ").unwrap();
        assert_eq!(
            updi.link.sent.as_slice(),
            &[0x55, 0xE0, b' ', b'g', b'o', b'r', b'P', b'M', b'V', b'N']
        );
    }

    #[test]
    fn ld_inc_repeats_then_reads() {
        let mut updi = Updi::new(RecordLink::new(&[1, 2, 3, 4]));
        let mut buf = [0u8; 4];
        updi.ld_inc(&mut buf).unwrap();
        assert_eq!(buf, [1, 2, 3, 4]);
        assert_eq!(
            updi.link.sent.as_slice(),
            &[0x55, 0xA0, 3, 0x55, 0x20 | (1 << 2)]
        );
    }

    #[test]
    fn st_inc_streams_with_per_byte_ack() {
        let mut updi = Updi::new(RecordLink::new(&[ACK, ACK, ACK]));
        updi.st_inc(&[0xAA, 0xBB, 0xCC]).unwrap();
        assert_eq!(
            updi.link.sent.as_slice(),
            &[0x55, 0xA0, 2, 0x55, 0x60 | (1 << 2), 0xAA, 0xBB, 0xCC]
        );
    }
}
