//! Internal helpers for bounded busy-wait loops.
//!
//! Two kinds of wait exist in the HAL. Loops that wait on an outside party (an
//! I2C slave, an incoming UART byte) can hang forever, so they take a timeout
//! and report it as an error. Loops the hardware always finishes on its own (a
//! byte shifting out, an oscillator settling) use [`spin_until`], which guards
//! against a broken or misconfigured peripheral with a generous budget and
//! panics rather than hanging silently.

/// Rough `CLK_PER` cycles per spin iteration (a status read plus the loop
/// branch). Only used to turn a millisecond timeout into an iteration count, so
/// it is a coarse estimate, not precise timing.
const CYCLES_PER_ITER: u32 = 8;

/// Iteration budget for the defensive guard on waits the hardware always
/// finishes. Far beyond any legitimate completion, so reaching it means the
/// peripheral is broken or misconfigured.
const DEFENSIVE_BUDGET: u32 = 1_000_000;

/// Turns a timeout in milliseconds into a spin-iteration budget for the given
/// `CLK_PER`. Returns at least 1. The result is a coarse upper bound on wall
/// time, not precise timing.
pub const fn budget_ms(f_cpu_hz: u32, timeout_ms: u32) -> u32 {
    let iters = (f_cpu_hz / 1000).saturating_mul(timeout_ms) / CYCLES_PER_ITER;
    if iters == 0 { 1 } else { iters }
}

/// Spins until `ready` returns true. Panics if the defensive budget is
/// exhausted, turning a silent hang into a loud abort.
#[inline]
pub fn spin_until(mut ready: impl FnMut() -> bool) {
    for _ in 0..DEFENSIVE_BUDGET {
        if ready() {
            return;
        }
    }
    panic!("avrxt-hal: blocking wait did not complete");
}
