//! TWI1 bus tests: address scan, expander readback, temperature sensor.
//!
//! Every test routes TWI1 to PB2/PB3 (PORTMUX ALT2) and re-inits the host
//! from scratch. The routing write is the point of `twi-scan`: cellcore
//! never sets it, so its bus silently sits on the PF2/PF3 default pins.
//!
//! Safety rule for the TCA9535 expanders: only the configuration register is
//! ever written. Driving U103 P12 (heartbeat) could arm the cellprog
//! recovery supervision, and P4/P5/P6 gate power hardware.

use avr_device::avr128da64 as pac;
use avrxt_hal::twi::{Error as TwiError, Twi};
use embedded_hal::i2c::I2c;
use hiltest_protocol::{AckList, Outcome, SCAN_FIRST, SCAN_LAST, SENTINEL};
use p3t1755::{Address as P3t1755Address, P3t1755, Temperature};
use tca9535::{Address as Tca9535Address, Configuration, Tca9535};
use ufmt::{uwrite, uwriteln};

use crate::context::Context;
use crate::detail::DetailBuf;

/// Bus clock. Matches cellcore.
const SCL_HZ: u32 = 100_000;

/// Per-wait timeout in nominal ms. `budget_ms` undercounts the real loop
/// cost by 5-12x, so one wait really lasts up to ~25 ms.
const TIMEOUT_MS: u32 = 2;

/// ACK set of a healthy, reworked bus: U103 (0x20), U1100 (0x21), U908
/// (0x42).
const EXPECTED_ACKS: [u8; 3] = [0x20, 0x21, 0x42];

/// All 16 expander pins as inputs: the power-on default and the only value
/// this image ever writes to an expander.
const ALL_INPUTS: Configuration = Configuration(0xFFFF);

/// Lower bound of the plausible bench temperature, in 1/16 degC.
const TEMP_MIN: Temperature = Temperature::from_degrees_celsius(5);
/// Upper bound of the plausible bench temperature, in 1/16 degC.
const TEMP_MAX: Temperature = Temperature::from_degrees_celsius(60);

/// Takes TWI1 and brings it up on PB2/PB3 at 100 kHz. The PORTMUX write is
/// the board fix under test: without ALT2 the signals never leave PF2/PF3.
fn bus(ctx: &mut Context) -> Twi<pac::TWI1> {
    let Some(twi1) = ctx.twi1.take() else {
        crate::halt()
    };
    ctx.portmux.twiroutea().modify(|_, w| w.twi1().alt2());
    Twi::with_timeout_ms(twi1, ctx.f_cpu.hz(), SCL_HZ, TIMEOUT_MS)
}

/// Probes every 7-bit address with a zero-length write (address-only, the
/// probe that proved reliable on this bus) and compares the ACK set against
/// [`EXPECTED_ACKS`]. The HAL issues a STOP even after a NACK, so each probe
/// leaves a clean bus for the next one.
pub fn scan<'a>(ctx: &mut Context, detail: &'a mut DetailBuf) -> (Outcome, Option<&'a str>) {
    let mut twi = bus(ctx);
    let mut acks = AckList::new();
    // Aborts are timeouts, bus errors, and lost arbitration: anything that
    // is neither an ACK nor a clean NACK.
    let mut aborts: u8 = 0;
    for addr in SCAN_FIRST..=SCAN_LAST {
        match twi.write(addr, &[]) {
            // The list capacity covers the whole scan range.
            Ok(()) => {
                acks.push(addr);
            }
            Err(TwiError::Nack) => {}
            Err(_) => aborts = aborts.saturating_add(1),
        }
    }
    ctx.twi1 = Some(twi.free());
    if aborts > 0 {
        let Ok(()) = uwriteln!(ctx.console, "{}log twi-scan aborts={}", SENTINEL, aborts);
    }
    let Ok(()) = uwrite!(detail, "{}", acks);
    let outcome = if acks.as_slice() == EXPECTED_ACKS.as_slice() {
        Outcome::Pass
    } else {
        Outcome::Fail
    };
    (outcome, Some(detail.as_str()))
}

/// Config-register write/readback on U103 (0x20) and U1100 (0x21). Output
/// and port registers are never touched, and every pin sits as an input
/// while the all-inputs value is in place.
pub fn tca9535_readback<'a>(
    ctx: &mut Context,
    detail: &'a mut DetailBuf,
) -> (Outcome, Option<&'a str>) {
    let mut twi = bus(ctx);
    let result = readback(&mut twi, Tca9535Address::Lll, "u103")
        .and_then(|()| readback(&mut twi, Tca9535Address::Llh, "u1100"));
    ctx.twi1 = Some(twi.free());
    match result {
        Ok(()) => (Outcome::Pass, None),
        Err((unit, reason)) => {
            let Ok(()) = uwrite!(detail, "{}-{}", unit, reason);
            (Outcome::Fail, Some(detail.as_str()))
        }
    }
}

/// One expander: write the all-inputs config, read it back, restore the
/// original, and verify the restore.
fn readback(
    twi: &mut Twi<pac::TWI1>,
    addr: Tca9535Address,
    unit: &'static str,
) -> Result<(), (&'static str, &'static str)> {
    let fail = |reason| (unit, reason);
    let mut expander = Tca9535::new(twi, addr);
    let original = expander
        .read_configuration()
        .map_err(|_| fail("config-read"))?;
    expander
        .write_configuration(ALL_INPUTS)
        .map_err(|_| fail("config-write"))?;
    let readback = expander
        .read_configuration()
        .map_err(|_| fail("readback-read"))?;
    expander
        .write_configuration(original)
        .map_err(|_| fail("config-restore"))?;
    let restored = expander
        .read_configuration()
        .map_err(|_| fail("restore-read"))?;
    if readback != ALL_INPUTS {
        return Err(fail("readback-mismatch"));
    }
    if restored != original {
        return Err(fail("restore-mismatch"));
    }
    Ok(())
}

/// Reads U908 (0x42) and expects a plausible bench temperature. The detail
/// reports the raw value in 1/16 degC units.
pub fn p3t1755_temp<'a>(
    ctx: &mut Context,
    detail: &'a mut DetailBuf,
) -> (Outcome, Option<&'a str>) {
    let mut twi = bus(ctx);
    let read = {
        let mut sensor = P3t1755::new(&mut twi, P3t1755Address::Addr3);
        sensor.read_temperature()
    };
    ctx.twi1 = Some(twi.free());
    let Ok(temperature) = read else {
        return (Outcome::Fail, Some("no-response"));
    };
    let raw = temperature.raw();
    let Ok(()) = uwrite!(detail, "raw={}", raw);
    let outcome = if (TEMP_MIN.raw()..=TEMP_MAX.raw()).contains(&raw) {
        Outcome::Pass
    } else {
        Outcome::Fail
    };
    (outcome, Some(detail.as_str()))
}
