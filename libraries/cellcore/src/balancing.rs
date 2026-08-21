//! The balancing-test subsystem: telemetry serving and actuator control.
//!
//! [`Balancing`] owns the commanded bleed state, serves the telemetry polls,
//! and drives the hardware through [`BalancingHw`].
//!
//! The bleed actuators start safe (masks and duty zero) and the refresh
//! timeout trips them back to zero when the host stops sending bleed
//! commands. The timeout is inert until the first command arms it.
//!
//! Temps slot 2 serves the routed cellagent sensor from a cache that
//! [`Balancing::note_agent_temp`] refreshes.

use cellguard_protocol::{
    CELLS, Kind, RAILS, RailSnapshot, Snapshot, TEMP_INVALID, TEMPS, TempSnapshot, decode_bleed,
    decode_pwm, encode_temps,
};

/// Caller ticks (any free-running unit) after which an unrefreshed bleed
/// command trips to the safe state.
pub const DEFAULT_BLEED_TIMEOUT_TICKS: u32 = 4096;

/// Missed routed temperature polls before the cached cellagent reading
/// expires to `TEMP_INVALID`.
const AGENT_TEMP_MISS_LIMIT: u8 = 3;

/// The hardware seam behind the balancing layer.
///
/// Snapshot reads fill the latest device-side values, so polling never
/// triggers a conversion burst.
pub trait BalancingHw {
    /// Writes the bleed-leg enable masks to the PWM expander.
    fn set_bleed(&mut self, en_3r6: u8, en_36r5: u8);
    /// Sets the bleed PWM duty in 1/65536 units. Zero disables modulation:
    /// the legs are statically on when enabled.
    fn set_pwm(&mut self, duty: u16);
    /// Writes the power-enable flags (`POWER_ACTIVE_BALANCER`,
    /// `POWER_EN_ALL`).
    fn set_power(&mut self, flags: u8);
    /// Asserts or releases the hardware gate-off line.
    fn set_gate_off(&mut self, on: bool);
    /// Fills the latest cell-voltage snapshot.
    fn cell_snapshot(&mut self, out: &mut Snapshot);
    /// Fills the latest balance-current snapshot.
    fn current_snapshot(&mut self, out: &mut Snapshot);
    /// Fills the latest rail snapshot.
    fn rails(&mut self, out: &mut RailSnapshot);
    /// Fills the latest temperature snapshot, `TEMP_INVALID` per missing
    /// sensor. The layer overwrites the routed-cellagent slot from its
    /// cache.
    fn temps(&mut self, out: &mut TempSnapshot);
    /// The `TINY_ALL_OFF` net readback.
    fn tiny_all_off(&mut self) -> bool;
    /// The `EMERGENCY_GATE_OFF` pin state.
    fn emergency_gate_off(&mut self) -> bool;
    /// Whether the cellagent link showed recent activity.
    fn cellagent_alive(&mut self) -> bool;
}

/// The balancing state layer. See the [module](self) docs.
pub struct Balancing<H: BalancingHw> {
    hw: H,
    en_3r6: u8,
    en_36r5: u8,
    duty: u16,
    gate_mask: u8,
    timeout_ticks: u32,
    last_refresh: u32,
    armed: bool,
    agent_temp: i16,
    agent_temp_misses: u8,
}

impl<H: BalancingHw> Balancing<H> {
    /// Creates the layer over `hw` with the default refresh timeout.
    #[must_use]
    pub const fn new(hw: H) -> Self {
        Self::with_timeout(hw, DEFAULT_BLEED_TIMEOUT_TICKS)
    }

    /// Creates the layer with an explicit refresh timeout in caller ticks.
    #[must_use]
    pub const fn with_timeout(hw: H, timeout_ticks: u32) -> Self {
        Self {
            hw,
            en_3r6: 0,
            en_36r5: 0,
            duty: 0,
            gate_mask: 0,
            timeout_ticks,
            last_refresh: 0,
            armed: false,
            agent_temp: TEMP_INVALID,
            agent_temp_misses: 0,
        }
    }

    /// Mutable access to the hardware, for duties the layer does not model
    /// (like the heartbeat cadence).
    pub const fn hw_mut(&mut self) -> &mut H {
        &mut self.hw
    }

    /// Releases the hardware, consuming the layer.
    #[must_use]
    pub fn free(self) -> H {
        self.hw
    }

    /// Records a gate mask routed to the cellagent, reported in
    /// [`Kind::BalancerStatus`].
    pub const fn note_gate_mask(&mut self, mask: u8) {
        self.gate_mask = mask;
    }

    /// Records one routed cellagent temperature poll. Consecutive misses
    /// expire the cache to `TEMP_INVALID`.
    pub const fn note_agent_temp(&mut self, temp: Option<i16>) {
        if let Some(centi) = temp {
            self.agent_temp = centi;
            self.agent_temp_misses = 0;
        } else {
            self.agent_temp_misses = self.agent_temp_misses.saturating_add(1);
            if self.agent_temp_misses >= AGENT_TEMP_MISS_LIMIT {
                self.agent_temp = TEMP_INVALID;
            }
        }
    }

    /// Drives the bleed safe when the refresh window elapsed. Call every
    /// loop iteration with the current tick.
    pub fn tick(&mut self, now: u32) {
        if self.armed && now.wrapping_sub(self.last_refresh) > self.timeout_ticks {
            // Masks first: a zero duty disables modulation (statically on),
            // so the mask write must land before the duty parks.
            self.hw.set_bleed(0, 0);
            self.hw.set_pwm(0);
            self.duty = 0;
            self.en_3r6 = 0;
            self.en_36r5 = 0;
            self.armed = false;
        }
    }

    /// Handles one balancing-test request. `now` stamps the refresh window
    /// of bleed commands. Returns `None` for kinds this layer does not own.
    pub fn handle(
        &mut self,
        now: u32,
        kind: Kind,
        payload: &[u8],
        out: &mut [u8],
    ) -> Option<(Kind, usize)> {
        match kind {
            Kind::ReadCellVoltages => {
                let mut snap = Snapshot {
                    seq: 0,
                    codes: [0; CELLS],
                };
                self.hw.cell_snapshot(&mut snap);
                let payload = snap.encode(out)?;
                Some((Kind::CellVoltages, payload.len()))
            }
            Kind::ReadBalanceCurrents => {
                let mut snap = Snapshot {
                    seq: 0,
                    codes: [0; CELLS],
                };
                self.hw.current_snapshot(&mut snap);
                let payload = snap.encode(out)?;
                Some((Kind::BalanceCurrents, payload.len()))
            }
            Kind::ReadRails => {
                let mut snap = RailSnapshot { codes: [0; RAILS] };
                self.hw.rails(&mut snap);
                let payload = snap.encode(out)?;
                Some((Kind::Rails, payload.len()))
            }
            Kind::ReadTemperatures => {
                let mut temps = [TEMP_INVALID; TEMPS];
                self.hw.temps(&mut temps);
                // Slot 2 is the routed cellagent sensor, served from the
                // poll cache.
                temps[2] = self.agent_temp;
                let payload = encode_temps(&temps, out)?;
                Some((Kind::Temperatures, payload.len()))
            }
            Kind::ReadBalancerStatus => {
                let status = cellguard_protocol::BalancerStatus {
                    en_3r6: self.en_3r6,
                    en_36r5: self.en_36r5,
                    pwm_duty: self.duty,
                    gate_mask: self.gate_mask,
                    tiny_all_off: self.hw.tiny_all_off(),
                    emergency_gate_off: self.hw.emergency_gate_off(),
                    active_balancer_on: false,
                    en_all: false,
                    cellagent_alive: self.hw.cellagent_alive(),
                };
                let payload = status.encode(out)?;
                Some((Kind::BalancerStatus, payload.len()))
            }
            Kind::SetBleed => {
                let masks = decode_bleed(payload)?;
                self.hw.set_bleed(masks.en_3r6, masks.en_36r5);
                self.en_3r6 = masks.en_3r6;
                self.en_36r5 = masks.en_36r5;
                self.last_refresh = now;
                self.armed = true;
                Some((Kind::Ack, 0))
            }
            Kind::SetBleedPwm => {
                let duty = decode_pwm(payload)?;
                self.hw.set_pwm(duty);
                self.duty = duty;
                self.last_refresh = now;
                self.armed = true;
                Some((Kind::Ack, 0))
            }
            Kind::SetPower => {
                let flags = *payload.first()?;
                self.hw.set_power(flags);
                Some((Kind::Ack, 0))
            }
            Kind::GateOff => {
                let on = *payload.first()?;
                self.hw.set_gate_off(on != 0);
                Some((Kind::Ack, 0))
            }
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use cellguard_protocol::{
        BalancerStatus, BleedMasks, POWER_ACTIVE_BALANCER, POWER_EN_ALL, Snapshot, decode_pwm,
    };

    use super::{Balancing, BalancingHw};

    #[derive(Default)]
    struct MockHw {
        bleed: (u8, u8),
        duty: u16,
        power: u8,
        gate_off: bool,
        cell_seq: u8,
        tiny_all_off: bool,
        agent_alive: bool,
    }

    impl BalancingHw for MockHw {
        fn set_bleed(&mut self, en_3r6: u8, en_36r5: u8) {
            self.bleed = (en_3r6, en_36r5);
        }
        fn set_pwm(&mut self, duty: u16) {
            self.duty = duty;
        }
        fn set_power(&mut self, flags: u8) {
            self.power = flags;
        }
        fn set_gate_off(&mut self, on: bool) {
            self.gate_off = on;
        }
        fn cell_snapshot(&mut self, out: &mut Snapshot) {
            self.cell_seq = self.cell_seq.wrapping_add(1);
            out.seq = self.cell_seq;
            out.codes = [100, 200, 300, 400];
        }
        fn current_snapshot(&mut self, out: &mut Snapshot) {
            self.cell_seq = self.cell_seq.wrapping_add(1);
            out.seq = self.cell_seq;
            out.codes = [-1, -2, -3, -4];
        }
        fn rails(&mut self, out: &mut cellguard_protocol::RailSnapshot) {
            out.codes = [1, 2, 3, 4, 5, 6, 7, 8];
        }
        fn temps(&mut self, out: &mut cellguard_protocol::TempSnapshot) {
            out[0] = 2500;
            out[1] = 2600;
            out[2] = cellguard_protocol::TEMP_INVALID;
        }
        fn tiny_all_off(&mut self) -> bool {
            self.tiny_all_off
        }
        fn emergency_gate_off(&mut self) -> bool {
            self.gate_off
        }
        fn cellagent_alive(&mut self) -> bool {
            self.agent_alive
        }
    }

    fn handle(bal: &mut Balancing<MockHw>, kind: cellguard_protocol::Kind, payload: &[u8]) {
        let mut out = [0u8; 32];
        let result = bal.handle(0, kind, payload, &mut out);
        assert!(result.is_some(), "kind must be handled: {kind:?}");
    }

    #[test]
    fn set_bleed_commands_and_refreshes() {
        let mut bal = Balancing::new(MockHw::default());
        handle(&mut bal, cellguard_protocol::Kind::SetBleed, &[0x0F, 0x01]);
        assert_eq!(bal.hw.bleed, (0x0F, 0x01));

        let mut out = [0u8; 32];
        assert!(
            bal.handle(0, cellguard_protocol::Kind::SetBleed, &[0x01], &mut out)
                .is_none()
        );
    }

    #[test]
    fn set_power_and_gate_off_map_flags() {
        let mut bal = Balancing::new(MockHw::default());
        handle(
            &mut bal,
            cellguard_protocol::Kind::SetPower,
            &[POWER_ACTIVE_BALANCER | POWER_EN_ALL],
        );
        assert_eq!(bal.hw.power, POWER_ACTIVE_BALANCER | POWER_EN_ALL);
        handle(&mut bal, cellguard_protocol::Kind::GateOff, &[1]);
        assert!(bal.hw.gate_off);
        handle(&mut bal, cellguard_protocol::Kind::GateOff, &[0]);
        assert!(!bal.hw.gate_off);
    }

    #[test]
    fn telemetry_reads_return_latest_snapshots() {
        let mut bal = Balancing::new(MockHw::default());
        let mut out = [0u8; 32];

        let (kind, len) = bal
            .handle(0, cellguard_protocol::Kind::ReadCellVoltages, &[], &mut out)
            .unwrap();
        assert_eq!(kind, cellguard_protocol::Kind::CellVoltages);
        let snap = Snapshot::decode(out.get(..len).unwrap()).unwrap();
        assert_eq!(snap.codes, [100, 200, 300, 400]);
        assert_eq!(snap.seq, 1);

        let (kind, len) = bal
            .handle(
                0,
                cellguard_protocol::Kind::ReadBalanceCurrents,
                &[],
                &mut out,
            )
            .unwrap();
        assert_eq!(kind, cellguard_protocol::Kind::BalanceCurrents);
        let snap = Snapshot::decode(out.get(..len).unwrap()).unwrap();
        assert_eq!(snap.codes, [-1, -2, -3, -4]);

        let (kind, len) = bal
            .handle(0, cellguard_protocol::Kind::ReadTemperatures, &[], &mut out)
            .unwrap();
        assert_eq!(kind, cellguard_protocol::Kind::Temperatures);
        let temps = cellguard_protocol::decode_temps(out.get(..len).unwrap()).unwrap();
        assert_eq!(temps[0], 2500);
        assert_eq!(temps[2], cellguard_protocol::TEMP_INVALID);
    }

    #[test]
    fn routed_agent_temp_serves_slot_2_with_staleness() {
        fn read_slot2(bal: &mut Balancing<MockHw>) -> i16 {
            let mut out = [0u8; 32];
            let (_, len) = bal
                .handle(0, cellguard_protocol::Kind::ReadTemperatures, &[], &mut out)
                .unwrap();
            let temps = cellguard_protocol::decode_temps(out.get(..len).unwrap()).unwrap();
            temps[2]
        }

        let mut bal = Balancing::new(MockHw::default());

        assert_eq!(read_slot2(&mut bal), cellguard_protocol::TEMP_INVALID);

        bal.note_agent_temp(Some(2100));
        assert_eq!(read_slot2(&mut bal), 2100);

        bal.note_agent_temp(None);
        bal.note_agent_temp(None);
        assert_eq!(read_slot2(&mut bal), 2100);

        bal.note_agent_temp(None);
        assert_eq!(read_slot2(&mut bal), cellguard_protocol::TEMP_INVALID);

        bal.note_agent_temp(Some(2050));
        bal.note_agent_temp(None);
        assert_eq!(read_slot2(&mut bal), 2050);
    }

    #[test]
    fn balancer_status_reports_commanded_and_sensed_state() {
        let mut bal = Balancing::new(MockHw::default());
        bal.hw.tiny_all_off = true;
        bal.hw.agent_alive = true;
        bal.note_gate_mask(0x03);
        handle(&mut bal, cellguard_protocol::Kind::SetBleed, &[0x0F, 0x00]);
        handle(
            &mut bal,
            cellguard_protocol::Kind::SetBleedPwm,
            &[0x00, 0x80],
        );

        let mut out = [0u8; 32];
        let (_, len) = bal
            .handle(
                0,
                cellguard_protocol::Kind::ReadBalancerStatus,
                &[],
                &mut out,
            )
            .unwrap();
        let status = BalancerStatus::decode(out.get(..len).unwrap()).unwrap();
        assert_eq!(status.en_3r6, 0x0F);
        assert_eq!(status.pwm_duty, 0x8000);
        assert_eq!(status.gate_mask, 0x03);
        assert!(status.tiny_all_off);
        assert!(status.cellagent_alive);
    }

    #[test]
    fn unknown_kinds_are_not_owned() {
        let mut bal = Balancing::new(MockHw::default());
        let mut out = [0u8; 32];
        assert!(
            bal.handle(0, cellguard_protocol::Kind::BootProbe, &[], &mut out)
                .is_none()
        );
    }

    #[test]
    fn bleed_timeout_trips_to_safe_state() {
        let mut bal = Balancing::with_timeout(MockHw::default(), 100);
        handle(&mut bal, cellguard_protocol::Kind::SetBleed, &[0x0F, 0x0F]);
        handle(
            &mut bal,
            cellguard_protocol::Kind::SetBleedPwm,
            &[0xFF, 0xFF],
        );

        // Silence from power-up never trips: nothing arms the timeout
        // before the first command.
        bal.tick(50);
        assert_eq!(bal.hw.duty, 0xFFFF);

        bal.tick(101);
        assert_eq!(bal.hw.duty, 0, "duty must trip to zero");
        assert_eq!(bal.hw.bleed, (0, 0), "masks must trip to zero");
        assert_eq!(bal.en_3r6, 0);

        bal.hw.duty = 0x1234;
        bal.tick(5000);
        assert_eq!(bal.hw.duty, 0x1234);
    }

    #[test]
    fn pwm_decode_is_little_endian() {
        assert_eq!(decode_pwm(&[0x34, 0x12]), Some(0x1234));
        let _ = BleedMasks {
            en_3r6: 0,
            en_36r5: 0,
        };
    }
}
