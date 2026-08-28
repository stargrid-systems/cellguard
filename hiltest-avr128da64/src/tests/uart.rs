//! UART echo tests: the host supplies a payload line and compares the echo.

use core::str;

use hiltest_protocol::{Event, Outcome, SENTINEL, TestId};
use ufmt::uwriteln;

use crate::context::Context;

/// Receive-timeout attempts while waiting for the payload line. Each attempt
/// is nominally 20 ms and really a few hundred ms, so the total stays well
/// inside the 8 s deadman.
const PAYLOAD_ATTEMPTS: u32 = 20;

/// Echo on the 4 MHz boot clock.
pub fn echo_rc(ctx: &mut Context) -> (Outcome, Option<&'static str>) {
    echo(ctx, TestId::UartEchoRc)
}

/// Echo after the external-clock switch. Skips when `clock-extclk` has not
/// passed this boot.
pub fn echo_24m(ctx: &mut Context) -> (Outcome, Option<&'static str>) {
    if !ctx.clock_switched {
        return (Outcome::Skip, Some("needs-clock-extclk"));
    }
    echo(ctx, TestId::UartEcho24m)
}

fn echo(ctx: &mut Context, id: TestId) -> (Outcome, Option<&'static str>) {
    // Prompt the host for the payload line.
    let Ok(()) = uwriteln!(ctx.console, "{}log {} send", SENTINEL, id.name());
    let mut buf = [0u8; 80];
    let Some(len) = ctx.console.read_line(&mut buf, PAYLOAD_ATTEMPTS) else {
        return (Outcome::Fail, Some("payload-timeout"));
    };
    let Some(payload) = buf.get(..len).and_then(|raw| str::from_utf8(raw).ok()) else {
        return (Outcome::Fail, Some("payload-not-utf8"));
    };
    let Ok(()) = uwriteln!(ctx.console, "{}", Event::Echo { payload });
    (Outcome::Pass, None)
}
