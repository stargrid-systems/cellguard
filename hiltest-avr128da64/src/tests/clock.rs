//! External-clock switch test.

use avrxt_hal::clock::{self, HfFreq};
use hiltest_protocol::{Outcome, SENTINEL};
use ufmt::uwriteln;

use crate::context::Context;

/// Switches the main clock to the 24 MHz external clock (Y100), verifies
/// `MCLKSTATUS.EXTS`, and re-inits the console divisor for the new
/// frequency. A panic inside `set_extclk` becomes a deferred result through
/// the resume record.
pub fn extclk(ctx: &mut Context) -> (Outcome, Option<&'static str>) {
    if ctx.clock_switched {
        return (Outcome::Pass, Some("already-switched"));
    }
    // Announce intent first: if the switch panics or hangs, the deferred
    // result still tells the host what was attempted.
    let Ok(()) = uwriteln!(ctx.console, "{}log clock-extclk switching", SENTINEL);
    // A byte still in the shifter would garble when the clock changes.
    ctx.console.flush();
    clock::set_extclk(&ctx.cpu, &ctx.clkctrl, HfFreq::Mhz24);
    if !ctx.clkctrl.mclkstatus().read().exts().bit_is_set() {
        return (Outcome::Fail, Some("exts-clear"));
    }
    ctx.f_cpu = HfFreq::Mhz24;
    ctx.clock_switched = true;
    ctx.console.set_f_cpu(ctx.f_cpu.hz());
    (Outcome::Pass, None)
}
