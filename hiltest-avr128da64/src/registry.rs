//! Test dispatch: maps a test id to its runner and wraps every run in the
//! watchdog deadman and the resume record.

use avr_device::avr128da64 as pac;
use avrxt_hal::wdt::{Period, Watchdog};
use hiltest_protocol::{Event, Outcome, TestId};
use ufmt::uwriteln;

use crate::context::Context;
use crate::resume;
use crate::tests::{clock, spi_eeprom, uart};

/// Runs `id`: emits the ack, arms the deadman and the resume record, and
/// emits exactly one result line.
pub fn run(ctx: &mut Context, id: TestId) {
    let Ok(()) = uwriteln!(ctx.console, "{}", Event::RunAck { id: id.name() });
    resume::arm(id);
    // SAFETY: the deadman is the only WDT user in this image, so the stolen
    // handle aliases nothing.
    let wdt = unsafe { pac::Peripherals::steal() }.WDT;
    // 8 s deadman: an unexpected hang becomes a watchdog reset, and the next
    // boot reports the armed test as failed.
    let deadman = Watchdog::start(&ctx.cpu, wdt, Period::Clk8k);
    let (outcome, detail) = dispatch(ctx, id);
    deadman.stop(&ctx.cpu);
    resume::disarm();
    let Ok(()) = uwriteln!(
        ctx.console,
        "{}",
        Event::Result {
            id: id.name(),
            outcome,
            detail,
        }
    );
}

fn dispatch(ctx: &mut Context, id: TestId) -> (Outcome, Option<&'static str>) {
    match id {
        TestId::UartEchoRc => uart::echo_rc(ctx),
        TestId::ClockExtclk => clock::extclk(ctx),
        TestId::UartEcho24m => uart::echo_24m(ctx),
        TestId::Spi0Cat25ProbeApp => spi_eeprom::probe_app(ctx),
    }
}
