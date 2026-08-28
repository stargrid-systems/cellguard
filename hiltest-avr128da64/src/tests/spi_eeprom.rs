//! SPI0 EEPROM probes and the factory identity read.
//!
//! The `cat25` driver keeps WREN private (its writes are self-contained), so
//! the non-destructive WEL round-trip talks to the device over the raw
//! `SpiBus` with a manually framed chip select. The identity read stays raw
//! for the same reason: the driver wants an `SpiDevice`, and one READ
//! command is smaller than the adapter.

use avr_device::avr128da64 as pac;
use avrxt_hal::gpio::Output;
use avrxt_hal::spi::{Error as SpiError, Prescaler, Spi};
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::{MODE_0, SpiBus};
use hiltest_protocol::{Outcome, SENTINEL, TestId};
use ufmt::{uwrite, uwriteln};

use crate::context::Context;
use crate::detail::DetailBuf;

const WREN: u8 = 0x06;
const WRDI: u8 = 0x04;
const RDSR: u8 = 0x05;
const READ: u8 = 0x03;
/// Write-enable latch bit in the status register.
const WEL: u8 = 0x02;

/// Factory identity record length (the `cellboot::factory` layout).
const RECORD_LEN: usize = 64;
/// Magic bytes at the start of the factory record.
const MAGIC: [u8; 4] = *b"CGID";

/// The EEPROM a probe talks to. All three share SPI0 and differ only in the
/// chip select.
#[derive(Clone, Copy)]
enum Chip {
    /// App staging EEPROM (CAT25M01, CS PG6).
    App,
    /// Boot EEPROM (CAT25128, CS PA7).
    Boot,
    /// Factory identity EEPROM (CAT25128, CS PG7).
    Ident,
}

/// Non-destructive probe of the app staging EEPROM (U104, CS PG6).
pub fn probe_app(ctx: &mut Context) -> (Outcome, Option<&'static str>) {
    probe(ctx, TestId::Spi0Cat25ProbeApp, Chip::App)
}

/// Non-destructive probe of the boot EEPROM (U105, CS PA7).
pub fn probe_boot(ctx: &mut Context) -> (Outcome, Option<&'static str>) {
    probe(ctx, TestId::Spi0Cat25ProbeBoot, Chip::Boot)
}

/// Non-destructive probe of the factory identity EEPROM (U106, CS PG7).
pub fn probe_ident(ctx: &mut Context) -> (Outcome, Option<&'static str>) {
    probe(ctx, TestId::Spi0Cat25ProbeIdent, Chip::Ident)
}

/// WREN must set the WEL bit and WRDI must clear it again. Nothing is
/// written to the array.
fn probe(ctx: &mut Context, id: TestId, chip: Chip) -> (Outcome, Option<&'static str>) {
    let Some(spi0) = ctx.spi0.take() else {
        crate::halt()
    };
    let mut spi = Spi::new(spi0, MODE_0, Prescaler::Div16);

    let (after_wren, after_wrdi) = {
        let cs = chip_select(ctx, chip);
        command(&mut spi, cs, WREN, &mut []);
        let mut after_wren = [0u8; 1];
        command(&mut spi, cs, RDSR, &mut after_wren);
        command(&mut spi, cs, WRDI, &mut []);
        let mut after_wrdi = [0u8; 1];
        command(&mut spi, cs, RDSR, &mut after_wrdi);
        (after_wren, after_wrdi)
    };

    ctx.spi0 = Some(spi.free());

    let [wren_sr] = after_wren;
    let [wrdi_sr] = after_wrdi;
    // Raw status bytes for diagnosis: 0xFF usually means a floating bus.
    let Ok(()) = uwrite!(ctx.console, "{}log {} sr=0x", SENTINEL, id.name());
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

/// Reads the 64-byte factory record from U106 and checks it in place: the
/// `CGID` magic and the CRC-32 over bytes 0..60. The full `cellboot` parser
/// adds nothing over this check that is worth pulling the crate into the
/// image. Read-only: the identity EEPROM is never written.
pub fn ident_read<'a>(ctx: &mut Context, detail: &'a mut DetailBuf) -> (Outcome, Option<&'a str>) {
    let Some(spi0) = ctx.spi0.take() else {
        crate::halt()
    };
    let mut spi = Spi::new(spi0, MODE_0, Prescaler::Div16);

    let mut record = [0u8; RECORD_LEN];
    let Ok(()) = ctx.cs_ident.set_low();
    // READ with a 16-bit big-endian address (CAT25128), from offset 0.
    ok(spi.write(&[READ, 0, 0]));
    ok(spi.read(&mut record));
    let Ok(()) = ctx.cs_ident.set_high();
    ctx.spi0 = Some(spi.free());

    if record.iter().all(|&byte| byte == 0xFF) {
        return (Outcome::Skip, Some("unprovisioned"));
    }
    let Some((body, crc_bytes)) = record.split_last_chunk::<4>() else {
        // Unreachable: the record is longer than the CRC.
        return (Outcome::Fail, Some("short-record"));
    };
    if record.first_chunk::<4>() != Some(&MAGIC) {
        return (Outcome::Fail, Some("bad-magic"));
    }
    if crc::checksum32(body) != u32::from_le_bytes(*crc_bytes) {
        return (Outcome::Fail, Some("bad-crc"));
    }
    // Board model, little-endian at 5..7 in the record.
    let model = body
        .get(5..7)
        .and_then(|bytes| bytes.try_into().ok())
        .map_or(0, u16::from_le_bytes);
    let Ok(()) = uwrite!(detail, "model=");
    for byte in model.to_be_bytes() {
        detail.write_hex_byte(byte);
    }
    (Outcome::Pass, Some(detail.as_str()))
}

/// The chip select line of `chip`.
const fn chip_select(ctx: &mut Context, chip: Chip) -> &mut Output {
    match chip {
        Chip::App => &mut ctx.cs_app,
        Chip::Boot => &mut ctx.cs_boot,
        Chip::Ident => &mut ctx.cs_ident,
    }
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
