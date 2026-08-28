#![no_std]
#![no_main]
#![expect(
    clippy::similar_names,
    reason = "port names mirror the silicon port letters"
)]

//! HIL test-runner firmware for the AVR128DA64.
//!
//! Standalone image linked at 0x0. It owns the whole flash, so the
//! bootloader is gone while it is installed: restore cellboot plus cellcore
//! after a bench session (`hiltest restore`).
//!
//! The runner boots on the internal 4 MHz RC oscillator, brings up USART5 on
//! PG4/PG5 (PORTMUX ALT1) at 115200, prints the boot banner and any deferred
//! verdict from the `.noinit` resume record, and then serves the
//! `hiltest-protocol` command loop: `PING`, `LIST`, `RUN <id>`, `REBOOT`.
//! Every run is wrapped in an 8 s watchdog deadman, and the panic handler
//! stores the panic location in the resume record before a software reset,
//! so a hang or panic inside a test is reported by the next boot banner
//! instead of killing the session.

use avr_device::avr128da64 as pac;
use avrxt_hal::clock::HfFreq;
use avrxt_hal::gpio::Port;
use avrxt_hal::rstctrl::RstInstance;
use hiltest_protocol::{Command, Event, SENTINEL, TestId};
use ufmt::{uwrite, uwriteln};

use self::console::Console;
use self::context::Context;

mod console;
mod context;
mod detail;
mod panic;
mod registry;
mod resume;
mod tests;

/// Console baud rate. The host must use the same value.
const BAUD: u32 = 115_200;

/// The boot clock. The main clock stays untouched until `clock-extclk` runs.
const BOOT_CLOCK: HfFreq = HfFreq::Mhz4;

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap_or_else(|| halt());
    let reset_flags = dp.RSTCTRL.flags();
    dp.RSTCTRL.clear(reset_flags);

    // USART5 on PG4/PG5 (ALT1), where the isolated debug console sits.
    dp.PORTMUX.usartrouteb().modify(|_, w| w.usart5().alt1());
    let portg = Port::new(dp.PORTG).split();
    let _bus_tx = portg.p4.into_output_high();
    let _bus_rx = portg.p5.into_input();
    let console = Console::new(dp.USART5, BOOT_CLOCK.hz(), BAUD);

    // SPI0 (PA4 MOSI, PA5 MISO, PA6 SCK) with the EEPROM chip selects on
    // PG6 (app), PA7 (boot), and PG7 (factory identity).
    let porta = Port::new(dp.PORTA).split();
    let _mosi = porta.p4.into_output();
    let _miso = porta.p5.into_input();
    let _sck = porta.p6.into_output();
    let cs_app = portg.p6.into_output_high();
    let cs_boot = porta.p7.into_output_high();
    let cs_ident = portg.p7.into_output_high();

    // TWI1 (PB2 SDA, PB3 SCL): expanders and the temperature sensor. The
    // internal pull-ups hold the bus between transactions.
    let portb = Port::new(dp.PORTB).split();
    let _sda = portb.p2.into_input_pullup();
    let _scl = portb.p3.into_input_pullup();

    let mut ctx = Context {
        console,
        cpu: dp.CPU,
        clkctrl: dp.CLKCTRL,
        portmux: dp.PORTMUX,
        spi0: Some(dp.SPI0),
        twi1: Some(dp.TWI1),
        cs_app,
        cs_boot,
        cs_ident,
        f_cpu: BOOT_CLOCK,
        clock_switched: false,
    };

    let Ok(()) = uwriteln!(
        ctx.console,
        "{}",
        Event::Boot {
            rstfr: reset_flags.bits(),
            clk: "rc4m",
        }
    );
    report_deferred(&mut ctx);
    let Ok(()) = uwriteln!(ctx.console, "{}", Event::Ready);

    let mut buf = [0u8; 96];
    loop {
        let Some(len) = ctx.console.read_line(&mut buf, u32::MAX) else {
            continue;
        };
        if len == 0 {
            continue;
        }
        let Some(raw) = buf.get(..len) else {
            continue;
        };
        let Ok(line) = core::str::from_utf8(raw) else {
            let Ok(()) = uwriteln!(ctx.console, "{}", Event::Err { reason: "not-utf8" });
            continue;
        };
        handle(&mut ctx, line);
    }
}

/// Executes one parsed command line.
fn handle(ctx: &mut Context, line: &str) {
    match Command::parse(line) {
        Ok(Command::Ping(n)) => {
            let Ok(()) = uwriteln!(ctx.console, "{}", Event::Pong(n));
        }
        Ok(Command::List) => {
            for id in TestId::ALL {
                let Ok(()) = uwriteln!(ctx.console, "{}", Event::Test { id: id.name() });
            }
        }
        Ok(Command::Run(name)) => match TestId::from_name(name) {
            Some(id) => registry::run(ctx, id),
            None => {
                let Ok(()) = uwriteln!(
                    ctx.console,
                    "{}",
                    Event::Err {
                        reason: "unknown-test",
                    }
                );
            }
        },
        Ok(Command::Reboot) => {
            ctx.console.flush();
            // SAFETY: the device resets on the next line, so the stolen
            // handle never aliases a live driver in any observable way.
            let dp = unsafe { pac::Peripherals::steal() };
            dp.RSTCTRL.software_reset(&dp.CPU)
        }
        Err(e) => {
            let Ok(()) = uwriteln!(ctx.console, "{}", Event::Err { reason: e.token() });
        }
    }
}

/// Reports the verdict the previous boot left in the resume record, if any.
fn report_deferred(ctx: &mut Context) {
    let Some(deferred) = resume::take() else {
        return;
    };
    match (deferred.test, deferred.panicked) {
        (Some(id), true) => {
            let Ok(()) = uwriteln!(
                ctx.console,
                "{}result {} FAIL panic:{}:{}",
                SENTINEL,
                id.name(),
                deferred.file(),
                deferred.line
            );
        }
        (Some(id), false) => {
            // A reset while a test was armed and no panic was recorded:
            // the deadman fired or the test reset the chip unexpectedly.
            let Ok(()) = uwriteln!(ctx.console, "{}result {} FAIL hang", SENTINEL, id.name());
        }
        (None, true) => {
            let Ok(()) = uwrite!(ctx.console, "{}log boot-panic ", SENTINEL);
            let Ok(()) = uwriteln!(ctx.console, "{}:{}", deferred.file(), deferred.line);
        }
        (None, false) => {
            let Ok(()) = uwriteln!(ctx.console, "{}log resume-garbled", SENTINEL);
        }
    }
}

/// Halts with interrupts disabled. Only for states where not even a panic
/// can be reported.
pub(crate) fn halt() -> ! {
    avr_device::interrupt::disable();
    loop {
        core::hint::spin_loop();
    }
}
