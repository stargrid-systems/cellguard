/// No operation.
pub const NULL: u16 = 0b0000_0000_0000_0000;
/// Reset the device.
pub const RESET: u16 = 0b0000_0000_0001_0001;
/// Place the device into standby mode.
pub const STANDBY: u16 = 0b0000_0000_0010_0010;
/// Wake the device from standby mode to conversion mode.
pub const WAKEUP: u16 = 0b0000_0000_0011_0011;
/// Lock the interface such that only the [`NULL`], [`UNLOCK`], and [`RREG`]
/// commands are valid.
pub const LOCK: u16 = 0b0000_0101_0101_0101;
/// Unlock the interface after the interface is locked.
pub const UNLOCK: u16 = 0b0000_0110_0110_0110;

/// The RREG is used to read the device registers.
pub const RREG: u16 = 0b1010_0000_0000_0000;

/// Read `n` contiguous registers starting at address `addr`.
pub const fn rreg(addr: u8, n: u8) -> u16 {
    xreg(RREG, addr as u16, n as u16)
}

/// The WREG command allows writing an arbitrary number of contiguous device
/// registers.
pub const WREG: u16 = 0b0110_0000_0000_0000;

/// Write `n` contiguous registers starting at address `addr`.
pub const fn wreg(addr: u8, n: u8) -> u16 {
    xreg(WREG, addr as u16, n as u16)
}

// 0bccca_aaaa_annn_nnnn
const ADDR_BITS: u16 = 7;
const N_BITS: u16 = 6;

const fn xreg(cmd: u16, addr: u16, n: u16) -> u16 {
    debug_assert!(addr < (1 << ADDR_BITS));
    debug_assert!((n - 1) < (1 << N_BITS));
    cmd | (addr << N_BITS) | (n - 1)
}
