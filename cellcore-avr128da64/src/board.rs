//! The board hardware behind the balancing-test telemetry.
//!
//! [`Board`] owns the I2C1 expanders (U103 power/heartbeat, U1100 bleed
//! enables), the rail ADC with its U100/U101 mux, the cellagent-liveness
//! inputs, and the emergency gate-off pin, and implements
//! [`BalancingHw`](cellcore::balancing::BalancingHw) over them. See
//! `scratch/hardware/balancing.md` for the netlist facts.
//!
//! The I2C devices share one TWI through transient borrows: each operation
//! wraps `&mut Twi` in a driver, runs one transaction, and drops it.
//!
//! Cell-voltage and balance-current snapshots are stubs in this revision:
//! the sequence advances per poll and the codes read zero. The ADS131M08
//! bring-up fills them.

use avr_device::avr128da64 as pac;
use avrxt_hal::adc::{Adc, Avr128Resolution, Prescaler as AdcPrescaler};
use avrxt_hal::gpio::{Input, Output};
use avrxt_hal::twi::Twi;
use avrxt_hal::vref::{Reference, Vref};
use cellcore::balancing::BalancingHw;

/// Heartbeat cadence in RTC ticks (~1.024 kHz): 256 ticks is about 250 ms.
const HEARTBEAT_TICKS: u16 = 256;
use cellguard_protocol::{RailSnapshot, Snapshot, TEMP_INVALID, TempSnapshot};
use embedded_hal::digital::{InputPin, OutputPin, StatefulOutputPin};
use p3t1755::P3t1755;
use tca9535::{Address, Configuration, Output as ExpanderOut, PinIndex as Pin, Tca9535};

/// U103 (I2C1 @0x20): power and heartbeat expander.
mod u103 {
    use tca9535::PinIndex;

    pub const WP_EEPROM_BOOT: PinIndex = PinIndex::P1;
    pub const WP_EEPROM_APP: PinIndex = PinIndex::P2;
    pub const ACTIVE_BALANCER_ON: PinIndex = PinIndex::P4;
    pub const EN_ALL: PinIndex = PinIndex::P5;
    /// Candidate `POWER_ON` driver per the Power sheet; bench-verify
    /// (see balancing.md).
    pub const POWER_ON: PinIndex = PinIndex::P6;
    pub const I2C_PWR_TEMP_EN: PinIndex = PinIndex::P11;
    pub const HEARTBEAT: PinIndex = PinIndex::P12;
}

/// U1100 (I2C1 @0x21) pin map, verified (see `scratch/hardware/balancing.md`).
mod u1100 {
    use tca9535::PinIndex;

    /// EN_3R6_1..4: leg-A (2.0 Ω) bleed enables, cells 1-4.
    pub const EN_3R6: [PinIndex; 4] = [PinIndex::P0, PinIndex::P1, PinIndex::P2, PinIndex::P3];
    /// EN_36R5_1..4: leg-B (7.2 Ω) bleed enables, cells 1-4.
    pub const EN_36R5: [PinIndex; 4] = [PinIndex::P10, PinIndex::P11, PinIndex::P12, PinIndex::P13];
    /// Static `PWM_SIGNAL` source. High = legs enabled (when masks allow).
    pub const PWM_STATIC: PinIndex = PinIndex::P5;
}

/// The board hardware. See the [module](self) docs.
pub struct Board {
    twi: Twi<pac::TWI1>,
    /// Cached U103 output register, so pin updates are one I2C write.
    power_out: ExpanderOut,
    /// Cached U1100 output register.
    bleed_out: ExpanderOut,
    /// U908's strapped address, once probed.
    temp_addr: Option<p3t1755::Address>,
    adc: Adc<pac::ADC0>,
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
    cell_seq: u8,
    current_seq: u8,
    last_heartbeat: u16,
}

impl Board {
    /// Brings up the board: expanders configured to safe defaults, ADC on the
    /// external 1.8 V reference, rail mux parked, `INA_EN` asserted.
    #[allow(clippy::too_many_arguments, reason = "hardware wiring")]
    pub fn new(
        mut twi: Twi<pac::TWI1>,
        vref: pac::VREF,
        adc0: pac::ADC0,
        mut _mux_a0: Output,
        mut mux_a1: Output,
        mut _mux_en: Output,
        mut ina_en: Output,
        gate_off: Output,
        tiny_all_off: Input,
        alive: Input,
    ) -> Self {
        let mut vref = Vref::new(vref);
        vref.set_adc0(Reference::External);
        let adc = Adc::new(adc0, AdcPrescaler::Div64, Avr128Resolution::Bits10);

        // Rail mux parked on the 5V0/3V3 position, enabled (active low).
        let _ = _mux_a0.set_low();
        let _ = mux_a1.set_low();
        let _ = _mux_en.set_low();
        // INA190 current-sense chain and the per-cell U204 muxes on.
        let _ = ina_en.set_high();

        // Safe power-up: EEPROMs write-protected, enables off, isolated
        // temp power on, heartbeat low.
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

        // Board bring-up writes are best-effort: a missing expander must not
        // brick the field-bus interface, and the status handlers report the
        // gap.
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
        // depending on board revision; probe both once.
        let mut probe =
            |addr: p3t1755::Address| P3t1755::new(&mut twi, addr).read_temperature().is_ok();
        let temp_addr = [p3t1755::Address::Addr2, p3t1755::Address::Addr3]
            .into_iter()
            .find(|addr| probe(*addr));

        Self {
            twi,
            power_out,
            bleed_out,
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
            cell_seq: 0,
            current_seq: 0,
            last_heartbeat: 0,
        }
    }

    /// Toggles the heartbeat pin on U103 P12 when the cadence elapsed.
    /// `now` is the caller's RTC tick; 256 ticks (about 250 ms) separate
    /// toggles, per the cellprog supervision contract.
    pub fn heartbeat(&mut self, now: u16) {
        if now.wrapping_sub(self.last_heartbeat) >= HEARTBEAT_TICKS {
            self.last_heartbeat = now;
            let next = !self.heartbeat_state();
            self.set_u103(u103::HEARTBEAT, next);
        }
    }

    /// The current heartbeat level.
    pub const fn heartbeat_state(&self) -> bool {
        self.power_out.0 & u103::HEARTBEAT.mask() != 0
    }

    /// Samples the cellagent ALIVE pin and records edges.
    pub fn poll_alive(&mut self) {
        let level = self.alive.is_high().unwrap_or(self.last_alive);
        if level != self.last_alive {
            self.last_alive = level;
            self.alive_edge_seen = true;
        }
    }

    /// Sets one U103 output pin through the cached register.
    fn set_u103(&mut self, pin: Pin, high: bool) {
        self.power_out = if high {
            self.power_out.with_high(pin)
        } else {
            self.power_out.with_low(pin)
        };
        let mut exp = Tca9535::new(&mut self.twi, Address::Lll);
        let _ = exp.write_output(self.power_out);
    }

    /// Sets one U1100 output pin through the cached register.
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
    /// [`5V0`, `3V3`, `1V8AN`, `3V3B`]; position 10 reads
    /// [`VBAT_A`, `VBAT_B`, `12V_CON`, `20V_MOS`] (MCU sheet).
    fn read_mux_position(&mut self, a1: bool, out: &mut [u8; 4]) {
        if a1 {
            let _ = self.mux_a1.set_high();
        } else {
            let _ = self.mux_a1.set_low();
        }
        for (channel, slot) in out.iter_mut().enumerate() {
            let code = self.adc.read_channel(channel as u8);
            *slot = u8::try_from(code).unwrap_or(0);
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

    /// Static interpretation: any nonzero duty enables the legs through the
    /// U1100 P05 `PWM_SIGNAL` source. Pulse-width modulation on PB7 (TCD0
    /// WOD) is a follow-up; the hardware accepts a static enable.
    fn set_pwm(&mut self, duty: u16) {
        self.set_u1100(u1100::PWM_STATIC, duty > 0);
    }

    fn set_power(&mut self, flags: u8) {
        self.set_u103(u103::ACTIVE_BALANCER_ON, flags & 0x01 != 0);
        self.set_u103(u103::EN_ALL, flags & 0x02 != 0);
        // Bit 2 is the POWER_ON candidate (bench-verify the exact pin, see
        // balancing.md).
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
        self.cell_seq = self.cell_seq.wrapping_add(1);
        out.seq = self.cell_seq;
        // ADS131M08 bring-up fills these; zeros until then.
        out.codes = [0; 4];
    }

    fn current_snapshot(&mut self, out: &mut Snapshot) {
        self.current_seq = self.current_seq.wrapping_add(1);
        out.seq = self.current_seq;
        out.codes = [0; 4];
    }

    fn rails(&mut self, out: &mut RailSnapshot) {
        let mut common = [0u8; 4];
        let mut vbat = [0u8; 4];
        self.read_mux_position(false, &mut common);
        self.read_mux_position(true, &mut vbat);
        // RAIL_ORDER: VBAT_A, VBAT_B, 5V0, 3V3, 3V3B, 1V8AN, 12V_CON, 20V_MOS.
        let codes = [
            u16::from(vbat[0]),
            u16::from(vbat[1]),
            u16::from(common[0]),
            u16::from(common[1]),
            u16::from(common[3]),
            u16::from(common[2]),
            u16::from(vbat[2]),
            u16::from(vbat[3]),
        ];
        for (slot, code) in out.codes.iter_mut().zip(codes) {
            *slot = code;
        }
    }

    fn temps(&mut self, out: &mut TempSnapshot) {
        *out = [TEMP_INVALID; 3];
        if let Some(addr) = self.temp_addr {
            let mut sensor = P3t1755::new(&mut self.twi, addr);
            if let Ok(temp) = sensor.read_temperature() {
                out[0] = temp.centi_degrees_celsius();
            }
        }
        // Slots 1 (ADC LM61) and 2 (routed cellagent LM61) fill in the
        // ADS131M08 and routed-query revisions.
    }

    fn tiny_all_off(&mut self) -> bool {
        self.tiny_all_off.is_low().unwrap_or(true)
    }

    fn emergency_gate_off(&mut self) -> bool {
        self.gate_off.is_set_high().unwrap_or(false)
    }

    fn cellagent_alive(&mut self) -> bool {
        self.poll_alive();
        self.alive_edge_seen
    }
}
