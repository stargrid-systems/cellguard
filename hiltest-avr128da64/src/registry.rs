//! Test dispatch: maps a test id to its runner and wraps every run in the
//! watchdog deadman and the resume record.

use avr_device::avr128da64 as pac;
use avrxt_hal::wdt::{Period, Watchdog};
use hiltest_protocol::{Event, Outcome, TestId};
use ufmt::uwriteln;

use crate::context::Context;
use crate::detail::DetailBuf;
use crate::resume;
use crate::tests::{clock, spi_eeprom, twi, uart};

/// Runs `id`: emits the ack, arms the deadman and the resume record, and
/// emits exactly one result line.
pub fn run(ctx: &mut Context, id: TestId) {
    let Ok(()) = uwriteln!(ctx.console, "{}", Event::RunAck { id: id.name() });
    resume::arm(id);
    // 8 s deadman: an unexpected hang becomes a watchdog reset, and the next
    // boot reports the armed test as failed.
    set_deadman(&ctx.cpu, Period::Clk8k);
    let mut detail_buf = DetailBuf::new();
    let (outcome, detail) = dispatch(ctx, id, &mut detail_buf);
    set_deadman(&ctx.cpu, Period::Off);
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

/// Sets the deadman period, waiting out `WDT.STATUS.SYNCBUSY` first.
///
/// A `CTRLA` write needs 2 to 3 cycles of the 1.024 kHz WDT clock to cross
/// into the WDT clock domain, and the hardware ignores `CTRLA` writes while
/// that is pending. A test that finished inside that ~3 ms window used to
/// lose its `Off` write here, leaving the 8 s deadman armed on an idle
/// board.
fn set_deadman(cpu: &pac::CPU, period: Period) {
    // SAFETY: the deadman is the only WDT user in this image, so the stolen
    // handle aliases nothing.
    let wdt = unsafe { pac::Peripherals::steal() }.WDT;
    while wdt.status().read().syncbusy().bit_is_set() {
        core::hint::spin_loop();
    }
    let _armed = Watchdog::start(cpu, wdt, period);
}

fn dispatch<'a>(
    ctx: &mut Context,
    id: TestId,
    detail: &'a mut DetailBuf,
) -> (Outcome, Option<&'a str>) {
    match id {
        TestId::UartEchoRc => uart::echo_rc(ctx),
        TestId::ClockExtclk => clock::extclk(ctx),
        TestId::UartEcho24m => uart::echo_24m(ctx),
        TestId::Spi0Cat25ProbeApp => spi_eeprom::probe_app(ctx),
        TestId::Spi0Cat25ProbeBoot => spi_eeprom::probe_boot(ctx),
        TestId::Spi0Cat25ProbeIdent => spi_eeprom::probe_ident(ctx),
        TestId::IdentRead => spi_eeprom::ident_read(ctx, detail),
        TestId::TwiScan => twi::scan(ctx, detail),
        TestId::Tca9535Readback => twi::tca9535_readback(ctx, detail),
        TestId::P3t1755Temp => twi::p3t1755_temp(ctx, detail),
    }
}
