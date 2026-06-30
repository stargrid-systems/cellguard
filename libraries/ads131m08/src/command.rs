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

/// Read `N` contiguous registers starting at address `addr`.
pub const fn rreg<const N: usize>(addr: u8) -> u16 {
    xreg::<N>(RREG, addr)
}

/// The WREG command allows writing an arbitrary number of contiguous device
/// registers.
pub const WREG: u16 = 0b0110_0000_0000_0000;

/// Write `N` contiguous registers starting at address `addr`.
pub const fn wreg<const N: usize>(addr: u8) -> u16 {
    xreg::<N>(WREG, addr)
}

// 0bccca_aaaa_annn_nnnn
const ADDR_BITS: u16 = 6;
const N_BITS: u16 = 7;

/// Returns a read / write register command for a block of `N` registers.
///
/// `N` is a const generic, so the count bound is checked at compile time. Any
/// caller that asks for zero or more than `2^N_BITS` registers fails to build.
const fn xreg<const N: usize>(cmd: u16, addr: u8) -> u16 {
    const { assert!(N >= 1 && N <= 1 << N_BITS, "register count out of range") };
    debug_assert!(
        (addr as u16) < (1 << ADDR_BITS),
        "register address out of range"
    );
    #[expect(
        clippy::cast_possible_truncation,
        reason = "the const assert above proves N <= 2^N_BITS, well within u16"
    )]
    let n = N as u16;
    cmd | ((addr as u16) << N_BITS) | (n - 1)
}
