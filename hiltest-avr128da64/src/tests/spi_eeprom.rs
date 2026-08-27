//! SPI0 EEPROM probes.
//!
//! The `cat25` driver keeps WREN private (its writes are self-contained), so
//! the non-destructive WEL round-trip talks to the device over the raw
//! `SpiBus` with a manually framed chip select.

use avr_device::avr128da64 as pac;
use avrxt_hal::gpio::Output;
use avrxt_hal::spi::{Error as SpiError, Prescaler, Spi};
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{MODE_0, SpiBus};
use hiltest_protocol::{Outcome, SENTINEL};
use ufmt::{uwrite, uwriteln};

use crate::context::Context;

const WREN: u8 = 0x06;
const WRDI: u8 = 0x04;
const RDSR: u8 = 0x05;
/// Write-enable latch bit in the status register.
const WEL: u8 = 0x02;

/// Non-destructive probe of the app staging EEPROM (CAT25M01, CS PG6):
/// WREN must set the WEL bit and WRDI must clear it again. Nothing is
/// written to the array.
pub fn probe_app(ctx: &mut Context) -> (Outcome, Option<&'static str>) {
    let Some(spi0) = ctx.spi0.take() else {
        crate::halt()
    };
    let mut spi = Spi::new(spi0, MODE_0, Prescaler::Div16);

    command(&mut spi, &mut ctx.cs_app, WREN, &mut []);
    let mut after_wren = [0u8; 1];
    command(&mut spi, &mut ctx.cs_app, RDSR, &mut after_wren);
    command(&mut spi, &mut ctx.cs_app, WRDI, &mut []);
    let mut after_wrdi = [0u8; 1];
    command(&mut spi, &mut ctx.cs_app, RDSR, &mut after_wrdi);

    ctx.spi0 = Some(spi.free());

    let [wren_sr] = after_wren;
    let [wrdi_sr] = after_wrdi;
    // Raw status bytes for diagnosis: 0xFF usually means a floating bus.
    let Ok(()) = uwrite!(ctx.console, "{}log spi0-cat25-probe-app sr=0x", SENTINEL);
    ctx.console.write_hex_byte(wren_sr);
    let Ok(()) = uwrite!(ctx.console, ",0x");
    ctx.console.write_hex_byte(wrdi_sr);
    let Ok(()) = uwriteln!(ctx.console, "");

    if wren_sr == 0xFF {
        return (Outcome::Fail, Some("bus-floating"));
    }
    if wren_sr & WEL == 0 {
        return (Outcome::Fail, Some("wel-not-set"));
    }
    if wrdi_sr & WEL != 0 {
        return (Outcome::Fail, Some("wel-not-cleared"));
    }
    (Outcome::Pass, None)
}

/// One CS-framed command: writes the opcode, then clocks `rx.len()` reply
/// bytes.
fn command(spi: &mut Spi<pac::SPI0>, cs: &mut Output, opcode: u8, rx: &mut [u8]) {
    let Ok(()) = cs.set_low();
    ok(spi.write(&[opcode]));
    ok(spi.read(rx));
    let Ok(()) = cs.set_high();
}

/// Unwraps an SPI result. The error type is uninhabited.
fn ok<T>(result: Result<T, SpiError>) -> T {
    match result {
        Ok(value) => value,
        Err(e) => match e {},
    }
}
