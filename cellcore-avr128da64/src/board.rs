//! Board hardware for the balancing-test telemetry. Netlist facts live in
//! `scratch/hardware/balancing.md`.
//!
//! The I2C devices share one TWI through transient borrows. The two
//! ADS131M08s share SPI1 through an init-once static cell.

use core::cell::RefCell;

use ads131m08::{Ads131m08, Config, Ready};
use avr_device::avr128da64 as pac;
use avrxt_hal::adc::{Adc as McuAdc, Avr128Resolution, Prescaler as AdcPrescaler};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::{Input, Output};
use avrxt_hal::spi::Spi;
use avrxt_hal::tcd::{Output as PwmOutput, Prescaler as PwmPrescaler, TcdPwm};
use avrxt_hal::twi::Twi;
use avrxt_hal::vref::{Reference, Vref};
use cellcore::balancing::BalancingHw;
use cellguard_protocol::{RailSnapshot, Snapshot, TEMP_INVALID, TEMPS, TempSnapshot};
use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use embedded_hal::spi::SpiDevice;
use embedded_hal_bus::spi::{NoDelay, RefCellDevice};

type Adc = Ads131m08<RefCellDevice<'static, Spi<pac::SPI1>, Output, NoDelay>, Ready>;
use p3t1755::P3t1755;
use tca9535::{Address, Configuration, Output as ExpanderOut, PinIndex as Pin, Tca9535};

/// Heartbeat cadence in RTC ticks (~1.024 kHz). 256 ticks is about 250 ms,
/// per the cellprog supervision contract.
const HEARTBEAT_TICKS: u16 = 256;

/// ALIVE freshness window in RTC ticks: four missed 256-tick toggles.
const ALIVE_WINDOW_TICKS: u16 = 1024;

const PWM_TOP: u16 = avrxt_hal::tcd::MAX_TOP;
/// Bleed-PWM clock: 24 MHz / 4 = 6 MHz, so the ramp modulates at ~1.47 kHz.
const PWM_PRESCALE: PwmPrescaler = PwmPrescaler::Div4;

const ADS_VREF_MV: i32 = 1200;
/// 24-bit two's-complement full scale.
const ADS_FULL_SCALE: i32 = 1 << 23;
/// LM61 transfer bias in millivolts (10 mV/degC above this).
const LM61_BIAS_MV: i32 = 300;

/// Shared SPI1 ADC bus, written once at boot. The firmware is single-threaded
/// and never enables interrupts, so access cannot race.
static mut ADC_SPI: Option<RefCell<Spi<pac::SPI1>>> = None;

fn adc_bus() -> &'static RefCell<Spi<pac::SPI1>> {
    // SAFETY: `ADC_SPI` is written exactly once (in `Board::new`) before
    // any call, and no interrupt or second thread exists to race the read,
    // so the reference is always valid here.
    let slot = unsafe { &*core::ptr::addr_of!(ADC_SPI) };
    slot.as_ref().unwrap_or_else(|| halt())
}

/// Configures one ADS131M08. Returns None when the chip does not answer, so
/// the rest of the board stays alive.
fn bring_up_adc<S: SpiDevice>(device: Ads131m08<S>) -> Option<Ads131m08<S, Ready>> {
    let mut device = device.configure(Config::default()).ok()?;
    device.wakeup().ok()?;
    let mut channels = [0i32; 8];
    device.read_data_after_pause(&mut channels).ok()?;
    Some(device)
}

/// U103 (I2C1 @0x20): power and heartbeat expander.
mod u103 {
    use tca9535::PinIndex;

    pub const WP_EEPROM_BOOT: PinIndex = PinIndex::P1;
    pub const WP_EEPROM_APP: PinIndex = PinIndex::P2;
    pub const ACTIVE_BALANCER_ON: PinIndex = PinIndex::P4;
    pub const EN_ALL: PinIndex = PinIndex::P5;
    /// Candidate `POWER_ON` driver per the Power sheet. Unverified, see
    /// balancing.md.
    pub const POWER_ON: PinIndex = PinIndex::P6;
    pub const I2C_PWR_TEMP_EN: PinIndex = PinIndex::P11;
    pub const HEARTBEAT: PinIndex = PinIndex::P12;
}

/// U1100 (I2C1 @0x21) pin map, verified (see `scratch/hardware/balancing.md`).
mod u1100 {
    use tca9535::PinIndex;

    /// Leg-A (2.0 ohm) bleed enables, cells 1-4.
    pub const EN_3R6: [PinIndex; 4] = [PinIndex::P0, PinIndex::P1, PinIndex::P2, PinIndex::P3];
    /// Leg-B (7.2 ohm) bleed enables, cells 1-4.
    pub const EN_36R5: [PinIndex; 4] = [PinIndex::P10, PinIndex::P11, PinIndex::P12, PinIndex::P13];
    /// Static `PWM_SIGNAL` source. High = legs enabled (when masks allow).
    pub const PWM_STATIC: PinIndex = PinIndex::P5;
}

/// Hardware for the balancing-test board.
///
/// Owns the I2C expanders, both ADS131M08s, the rail mux, and the bleed PWM,
/// plus the cached snapshots and supervision state built on them. Build one
/// with [`Board::new`].
pub struct Board {
    twi: Twi<pac::TWI1>,
    /// Cached U103 output register, so pin updates are one I2C write.
    power_out: ExpanderOut,
    /// Cached U1100 output register.
    bleed_out: ExpanderOut,
    /// Bleed PWM on TCD0 WOD (PB7 through PORTMUX ALT1).
    bleed_pwm: TcdPwm<pac::TCD0>,
    /// U908's strapped address, once probed.
    temp_addr: Option<p3t1755::Address>,
    adc: McuAdc<pac::ADC0>,
    /// U100/U101 rail-mux select pins. The scan uses A1 only (both scanned
    /// positions have A0 low) and leaves the mux enabled.
    _mux_a0: Output,
    mux_a1: Output,
    _mux_en: Output,
    /// PB5 `EMERGENCY_GATE_OFF`.
    gate_off: Output,
    /// PC7 `TINY_ALL_OFF` readback.
    tiny_all_off: Input,
    /// PG0 `TINY_ALIVE`.
    alive: Input,
    last_alive: bool,
    alive_edge_seen: bool,
    last_alive_edge: u16,
    now: u16,
    last_heartbeat: u16,
    /// ADC A (U800): cell voltages on ch0-3, LM61 on ch4.
    adc_a: Option<Adc>,
    /// ADC B (U801): balance currents on ch0-3 (IR mux position 0).
    adc_b: Option<Adc>,
    drdy_a: Input,
    drdy_b: Input,
    cells: Snapshot,
    currents: Snapshot,
    lm61_centi: i16,
}

impl Board {
    /// Brings up the board: expanders at safe defaults, ADC on the external
    /// 1.8 V reference, rail mux parked, `INA_EN` asserted.
    #[expect(clippy::too_many_arguments, reason = "hardware wiring")]
    #[expect(
        clippy::used_underscore_binding,
        reason = "underscore marks pins parked after bring-up and held only for ownership"
    )]
    pub fn new(
        mut twi: Twi<pac::TWI1>,
        cpu: &pac::CPU,
        vref: pac::VREF,
        adc0: pac::ADC0,
        tcd0: pac::TCD0,
        delay: &mut Delay,
        spi1: Spi<pac::SPI1>,
        mut adc_sync: Output,
        mut cs_adc_a: Output,
        mut cs_adc_b: Output,
        mut ir_a0: Output,
        mut ir_a1: Output,
        mut _mux_a0: Output,
        mut mux_a1: Output,
        mut _mux_en: Output,
        mut ina_en: Output,
        gate_off: Output,
        tiny_all_off: Input,
        alive: Input,
        drdy_a: Input,
        drdy_b: Input,
    ) -> Self {
        let mut vref = Vref::new(vref);
        vref.set_adc0(Reference::External);
        let adc = McuAdc::new(adc0, AdcPrescaler::Div64, Avr128Resolution::Bits10);
        // Bleed PWM starts at zero duty, so the legs stay off until commanded.
        let bleed_pwm = TcdPwm::new(cpu, tcd0, PWM_TOP, PWM_PRESCALE, PwmOutput::Wod);

        // Rail mux parked on the 5V0/3V3 position, enabled (active low).
        let _ = _mux_a0.set_low();
        let _ = mux_a1.set_low();
        let _ = _mux_en.set_low();
        // INA190 current-sense chain and the per-cell U204 muxes on.
        let _ = ina_en.set_high();
        // IR mux at position 00: INA190 balance currents on ADC B ch0-3.
        let _ = ir_a0.set_low();
        let _ = ir_a1.set_low();

        // Both ADS131M08s on SPI1, mode 1, through the static cell. A shared
        // SYNC/RESET pulse (PF3) realigns them. A missing chip parks as None.
        let _ = cs_adc_a.set_high();
        let _ = cs_adc_b.set_high();
        {
            // SAFETY: written exactly once, before any `adc_bus()` call.
            unsafe {
                *core::ptr::addr_of_mut!(ADC_SPI) = Some(RefCell::new(spi1));
            }
        }
        let _ = ads131m08::pulse_reset(&mut adc_sync, delay);
        let dev_a = RefCellDevice::new_no_delay(adc_bus(), cs_adc_a).unwrap_or_else(|_| halt());
        let dev_b = RefCellDevice::new_no_delay(adc_bus(), cs_adc_b).unwrap_or_else(|_| halt());
        let adc_a = bring_up_adc(Ads131m08::new(dev_a));
        let adc_b = bring_up_adc(Ads131m08::new(dev_b));

        // Safe power-up: EEPROMs write-protected, enables off, temp power
        // isolated, heartbeat low.
        let power_out = ExpanderOut(0x0000)
            .with_high(u103::WP_EEPROM_BOOT)
            .with_high(u103::WP_EEPROM_APP)
            .with_low(u103::I2C_PWR_TEMP_EN);
        let bleed_out = ExpanderOut(0x0000);

        // U103: P00-P12 outputs (P13-P17 stay inputs).
        let power_config = Configuration(0x0000).with_input(Pin::P13);
        // U1100: P00-P05 and P10-P13 outputs.
        let bleed_config = Configuration(0x0000)
            .with_input(Pin::P6)
            .with_input(Pin::P7);

        // Bring-up writes are best-effort: a missing expander must not brick
        // the field-bus interface.
        {
            let mut exp = Tca9535::new(&mut twi, Address::Lll);
            let _ = exp.write_configuration(power_config);
            let _ = exp.write_output(power_out);
        }
        {
            let mut exp = Tca9535::new(&mut twi, Address::Llh);
            let _ = exp.write_configuration(bleed_config);
            let _ = exp.write_output(bleed_out);
        }

        // U908 P3T1755 on I2C1. The strapped address is 0x41 or 0x42
        // depending on board revision. Probe both once.
        let mut probe =
            |addr: p3t1755::Address| P3t1755::new(&mut twi, addr).read_temperature().is_ok();
        let temp_addr = [p3t1755::Address::Addr2, p3t1755::Address::Addr3]
            .into_iter()
            .find(|addr| probe(*addr));

        Self {
            twi,
            power_out,
            bleed_out,
            bleed_pwm,
            temp_addr,
            adc,
            _mux_a0,
            mux_a1,
            _mux_en,
            gate_off,
            tiny_all_off,
            alive,
            last_alive: false,
            alive_edge_seen: false,
            last_alive_edge: 0,
            now: 0,
            last_heartbeat: 0,
            adc_a,
            adc_b,
            drdy_a,
            drdy_b,
            cells: Snapshot {
                seq: 0,
                codes: [0; 4],
            },
            currents: Snapshot {
                seq: 0,
                codes: [0; 4],
            },
            lm61_centi: TEMP_INVALID,
        }
    }

    /// Toggles U103 P12 when the cadence elapsed. `now` is the RTC tick.
    pub fn heartbeat(&mut self, now: u16) {
        if now.wrapping_sub(self.last_heartbeat) >= HEARTBEAT_TICKS {
            self.last_heartbeat = now;
            let next = !self.heartbeat_state();
            self.set_u103(u103::HEARTBEAT, next);
        }
    }

    /// Returns the current U103 P12 heartbeat level, from the cached output
    /// register rather than the chip.
    pub const fn heartbeat_state(&self) -> bool {
        self.power_out.0 & u103::HEARTBEAT.mask() != 0
    }

    /// Polls the ADS131M08 data-ready lines and reads waiting samples into
    /// the snapshot buffers. A read takes one SPI frame (~100 us at 3 MHz).
    pub fn poll_adcs(&mut self) {
        if self.drdy_a.is_low().unwrap_or(false)
            && let Some(adc) = self.adc_a.as_mut()
        {
            let mut channels = [0i32; 8];
            if adc.read_data(&mut channels).is_ok() {
                self.cells.seq = self.cells.seq.wrapping_add(1);
                for (slot, code) in self.cells.codes.iter_mut().zip(channels) {
                    *slot = code;
                }
                // LM61 board temperature rides ch4-7 (tied together).
                self.lm61_centi = lm61_centi(channels[4]);
            }
        }
        if self.drdy_b.is_low().unwrap_or(false)
            && let Some(adc) = self.adc_b.as_mut()
        {
            let mut channels = [0i32; 8];
            if adc.read_data(&mut channels).is_ok() {
                self.currents.seq = self.currents.seq.wrapping_add(1);
                for (slot, code) in self.currents.codes.iter_mut().zip(channels) {
                    *slot = code;
                }
            }
        }
    }

    /// Samples the PG0 `TINY_ALIVE` line and latches edge timing for the
    /// freshness check. `now` is the RTC tick.
    pub fn poll_alive(&mut self, now: u16) {
        self.now = now;
        let level = self.alive.is_high().unwrap_or(self.last_alive);
        if level != self.last_alive {
            self.last_alive = level;
            self.alive_edge_seen = true;
            self.last_alive_edge = now;
        }
    }

    fn set_u103(&mut self, pin: Pin, high: bool) {
        self.power_out = if high {
            self.power_out.with_high(pin)
        } else {
            self.power_out.with_low(pin)
        };
        let mut exp = Tca9535::new(&mut self.twi, Address::Lll);
        let _ = exp.write_output(self.power_out);
    }

    fn set_u1100(&mut self, pin: Pin, high: bool) {
        self.bleed_out = if high {
            self.bleed_out.with_high(pin)
        } else {
            self.bleed_out.with_low(pin)
        };
        let mut exp = Tca9535::new(&mut self.twi, Address::Llh);
        let _ = exp.write_output(self.bleed_out);
    }

    /// Reads one rail-mux position into `out` (AIN0-3). Position 00 reads
    /// [`5V0`, `3V3`, `1V8AN`, `3V3B`]. Position 10 reads
    /// [`VBAT_A`, `VBAT_B`, `12V_CON`, `20V_MOS`] (MCU sheet).
    fn read_mux_position(&mut self, a1: bool, out: &mut [u16; 4]) {
        if a1 {
            let _ = self.mux_a1.set_high();
        } else {
            let _ = self.mux_a1.set_low();
        }
        for (slot, channel) in out.iter_mut().zip([0u8, 1, 2, 3]) {
            *slot = self.adc.read_channel(channel);
        }
    }
}

impl BalancingHw for Board {
    fn set_bleed(&mut self, en_3r6: u8, en_36r5: u8) {
        for (i, pin) in u1100::EN_3R6.iter().enumerate() {
            self.set_u1100(*pin, en_3r6 & (1 << i) != 0);
        }
        for (i, pin) in u1100::EN_36R5.iter().enumerate() {
            self.set_u1100(*pin, en_36r5 & (1 << i) != 0);
        }
    }

    /// Duty 0 statically enables the legs through the U1100 P05 `PWM_SIGNAL`
    /// source while PB7 parks low, per the protocol. Any other duty
    /// modulates PB7 and drops P05. The sources are `ORed` in hardware.
    fn set_pwm(&mut self, duty: u16) {
        if duty == 0 {
            self.bleed_pwm.set_on_ticks(0);
            self.set_u1100(u1100::PWM_STATIC, true);
        } else {
            self.set_u1100(u1100::PWM_STATIC, false);
            self.bleed_pwm.set_on_ticks(pwm_ticks(duty));
        }
    }

    fn set_power(&mut self, flags: u8) {
        self.set_u103(u103::ACTIVE_BALANCER_ON, flags & 0x01 != 0);
        self.set_u103(u103::EN_ALL, flags & 0x02 != 0);
        // Bit 2 is the POWER_ON candidate. Unverified pin, see balancing.md.
        self.set_u103(u103::POWER_ON, flags & 0x04 != 0);
    }

    fn set_gate_off(&mut self, on: bool) {
        if on {
            let _ = self.gate_off.set_high();
        } else {
            let _ = self.gate_off.set_low();
        }
    }

    fn cell_snapshot(&mut self, out: &mut Snapshot) {
        *out = self.cells;
    }

    fn current_snapshot(&mut self, out: &mut Snapshot) {
        *out = self.currents;
    }

    fn rails(&mut self, out: &mut RailSnapshot) {
        let mut common = [0u16; 4];
        let mut vbat = [0u16; 4];
        self.read_mux_position(false, &mut common);
        self.read_mux_position(true, &mut vbat);
        // RAIL_ORDER: VBAT_A, VBAT_B, 5V0, 3V3, 3V3B, 1V8AN, 12V_CON, 20V_MOS.
        let codes = [
            vbat[0], vbat[1], common[0], common[1], common[3], common[2], vbat[2], vbat[3],
        ];
        for (slot, code) in out.codes.iter_mut().zip(codes) {
            *slot = code;
        }
    }

    fn temps(&mut self, out: &mut TempSnapshot) {
        out.temps = [TEMP_INVALID; TEMPS];
        if let Some(addr) = self.temp_addr {
            let mut sensor = P3t1755::new(&mut self.twi, addr);
            if let Ok(temp) = sensor.read_temperature() {
                out.temps[0] = temp.centi_degrees_celsius();
            }
        }
        out.temps[1] = self.lm61_centi;
        // Slot 2 is served by the balancing layer from its routed-poll
        // cache.
    }

    fn tiny_all_off(&mut self) -> bool {
        self.tiny_all_off.is_low().unwrap_or(true)
    }

    fn emergency_gate_off(&mut self) -> bool {
        self.gate_off.is_set_high().unwrap_or(false)
    }

    fn cellagent_alive(&mut self) -> bool {
        self.poll_alive(self.now);
        self.alive_edge_seen && self.now.wrapping_sub(self.last_alive_edge) < ALIVE_WINDOW_TICKS
    }
}

/// Scales a 1/65536-unit duty onto the TCD0 ramp, rounded to the nearest
/// tick.
fn pwm_ticks(duty: u16) -> u16 {
    u16::try_from((u32::from(duty) * u32::from(PWM_TOP) + 32_767) / 65_535).unwrap_or(PWM_TOP)
}

/// Converts an ADS131M08 word from the LM61 (10 mV/degC, 300 mV bias) to
/// centi-degrees Celsius. Gain is 1, reference 1.2 V.
fn lm61_centi(code: i32) -> i16 {
    let mv = code.saturating_mul(ADS_VREF_MV) / ADS_FULL_SCALE;
    i16::try_from(mv.saturating_sub(LM61_BIAS_MV).saturating_mul(10)).unwrap_or(TEMP_INVALID)
}

/// Halts with interrupts disabled. Unrecoverable board wiring failure.
fn halt() -> ! {
    avr_device::interrupt::disable();
    loop {
        core::hint::spin_loop();
    }
}
