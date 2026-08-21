//! Payload codecs for the balancing-test kinds.
//!
//! All readings are raw ADC codes, not engineering units. The conversion
//! constants live with the test tooling so a recalibration needs no firmware
//! update. They are documented here for the host side:
//!
//! - Cell voltages (ADC A ch0-3): `V_cell = code / 2^23 * VREF / 0.0330`
//!   (820k:28k divider, ADS131M08 1.2 V internal reference, gain 1). Valid only
//!   while `POWER_ON` is asserted.
//! - Balance currents (ADC B ch0-3, IR mux position 0): INA190A1 over a 47
//!   milliohm shunt, gain 25, so `I = V_out / (1.175 V/A)` where `V_out`
//!   converts from the code like a cell voltage. The `INA_REF` switch `S501`
//!   selects the range: GND is unipolar, 3.15 V is bipolar (subtract the REF
//!   midpoint before scaling).
//! - Rails: 10-bit MCU ADC with a 1.8 V reference. Divider scale: VBAT A/B =
//!   0.052, `3V3`/`3V3B`/`5V0`/`12V_CON`/`20V_MOS` = 0.0536, `1V8AN` = 0.5.
//! - Temperatures: centi-degrees Celsius, measured at the source.
//!
//! Payloads are little-endian throughout.

/// Number of cell channels reported in a snapshot.
pub const CELLS: usize = 4;

/// Number of rails reported by [`Kind::ReadRails`](crate::Kind::ReadRails).
pub const RAILS: usize = 8;

/// Number of temperature sensors reported by
/// [`Kind::ReadTemperatures`](crate::Kind::ReadTemperatures).
pub const TEMPS: usize = 3;

/// Rail order in a [`Kind::Rails`](crate::Kind::Rails) payload.
pub const RAIL_ORDER: [&str; RAILS] = [
    "VBAT_A", "VBAT_B", "5V0", "3V3", "3V3B", "1V8AN", "12V_CON", "20V_MOS",
];

/// Temperature-sensor order in a
/// [`Kind::Temperatures`](crate::Kind::Temperatures) payload.
pub const TEMP_ORDER: [&str; TEMPS] = ["main_p3t1755", "adc_lm61", "cellagent_lm61"];

/// Reported by [`Kind::Temperatures`](crate::Kind::Temperatures) for a
/// sensor that could not be read.
pub const TEMP_INVALID: i16 = i16::MIN;

/// Snapshot sequence counter type shared by the snapshot replies.
pub type Seq = u8;

/// Decoded cell-voltage or balance-current snapshot.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Snapshot {
    /// Increments per device-side snapshot. A repeat flags a stale read, a
    /// jump flags missed snapshots.
    pub seq: Seq,
    /// Raw ADC codes, cell 1 first. 24-bit values sign-extended to `i32`.
    pub codes: [i32; CELLS],
}

impl Snapshot {
    /// Payload length of the encoded form.
    pub const PAYLOAD_LEN: usize = 1 + CELLS * 4;

    /// Encodes into `out`, returning the payload slice.
    #[must_use]
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> Option<&'a [u8]> {
        let buf = out.get_mut(..Self::PAYLOAD_LEN)?;
        let (head, codes) = buf.split_first_mut()?;
        *head = self.seq;
        for (slot, code) in codes.chunks_exact_mut(4).zip(self.codes) {
            slot.copy_from_slice(&code.to_le_bytes());
        }
        out.get(..Self::PAYLOAD_LEN)
    }

    /// Decodes a payload into a snapshot.
    #[must_use]
    pub fn decode(payload: &[u8]) -> Option<Self> {
        let (seq, codes) = payload.split_first_chunk::<1>()?;
        if codes.len() != CELLS * 4 {
            return None;
        }
        let mut out = [0i32; CELLS];
        for (slot, code) in out.iter_mut().zip(codes.chunks_exact(4)) {
            let bytes: [u8; 4] = code.try_into().ok()?;
            *slot = i32::from_le_bytes(bytes);
        }
        Some(Self {
            seq: seq[0],
            codes: out,
        })
    }
}

/// Decoded rail snapshot: raw 10-bit MCU-ADC codes in [`RAIL_ORDER`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RailSnapshot {
    /// Raw codes, one per rail.
    pub codes: [u16; RAILS],
}

impl RailSnapshot {
    /// Payload length of the encoded form.
    pub const PAYLOAD_LEN: usize = RAILS * 2;

    /// Encodes into `out`, returning the payload slice.
    #[must_use]
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> Option<&'a [u8]> {
        let buf = out.get_mut(..Self::PAYLOAD_LEN)?.chunks_exact_mut(2);
        for (slot, code) in buf.zip(self.codes) {
            slot.copy_from_slice(&code.to_le_bytes());
        }
        out.get(..Self::PAYLOAD_LEN)
    }

    /// Decodes a payload into a rail snapshot.
    #[must_use]
    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != Self::PAYLOAD_LEN {
            return None;
        }
        let mut codes = [0u16; RAILS];
        for (slot, code) in codes.iter_mut().zip(payload.chunks_exact(2)) {
            let bytes: [u8; 2] = code.try_into().ok()?;
            *slot = u16::from_le_bytes(bytes);
        }
        Some(Self { codes })
    }
}

/// Decoded temperature frame: centi-degrees Celsius in [`TEMP_ORDER`].
pub type TempSnapshot = [i16; TEMPS];

/// Encodes a temperature snapshot into `out`, returning the payload slice.
#[must_use]
pub fn encode_temps<'a>(temps: &TempSnapshot, out: &'a mut [u8]) -> Option<&'a [u8]> {
    let len = TEMPS * 2;
    for (slot, temp) in out.get_mut(..len)?.chunks_exact_mut(2).zip(temps) {
        slot.copy_from_slice(&temp.to_le_bytes());
    }
    out.get(..len)
}

/// Decodes a temperature snapshot payload.
#[must_use]
pub fn decode_temps(payload: &[u8]) -> Option<TempSnapshot> {
    if payload.len() != TEMPS * 2 {
        return None;
    }
    let mut temps = [0i16; TEMPS];
    for (slot, temp) in temps.iter_mut().zip(payload.chunks_exact(2)) {
        let bytes: [u8; 2] = temp.try_into().ok()?;
        *slot = i16::from_le_bytes(bytes);
    }
    Some(temps)
}

/// Decoded [`Kind::BalancerStatus`](crate::Kind::BalancerStatus) payload.
///
/// One poll reports the full balancing state, commanded and actual.
#[expect(
    clippy::struct_excessive_bools,
    reason = "independent status bits, not a state machine"
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BalancerStatus {
    /// `en_3r6` mask last written to the PWM expander, bit x = cell x+1.
    pub en_3r6: u8,
    /// `en_36r5` mask last written to the PWM expander.
    pub en_36r5: u8,
    /// PWM duty in 1/65536 units currently latched.
    pub pwm_duty: u16,
    /// Gate mask the cellagent last reported (routed echo).
    pub gate_mask: u8,
    /// `TINY_ALL_OFF` net readback: true when the cellagent asserts all-off.
    pub tiny_all_off: bool,
    /// `EMERGENCY_GATE_OFF` (PB5) assertion state.
    pub emergency_gate_off: bool,
    /// `ACTIVE_BALANCER_ON` expander output state.
    pub active_balancer_on: bool,
    /// `EN_ALL` expander output state.
    pub en_all: bool,
    /// Whether the cellagent link is alive (recent activity).
    pub cellagent_alive: bool,
}

impl BalancerStatus {
    /// Payload length of the encoded form.
    pub const PAYLOAD_LEN: usize = 8;

    /// Encodes into `out`, returning the payload slice.
    #[must_use]
    pub fn encode<'a>(&self, out: &'a mut [u8]) -> Option<&'a [u8]> {
        let buf = out.get_mut(..Self::PAYLOAD_LEN)?;
        let (head, tail) = buf.split_at_mut(5);
        head.copy_from_slice(&[
            self.en_3r6,
            self.en_36r5,
            self.pwm_duty.to_le_bytes()[0],
            self.pwm_duty.to_le_bytes()[1],
            self.gate_mask,
        ]);
        tail.copy_from_slice(&[
            u8::from(self.tiny_all_off)
                | u8::from(self.emergency_gate_off) << 1
                | u8::from(self.active_balancer_on) << 2
                | u8::from(self.en_all) << 3
                | u8::from(self.cellagent_alive) << 4,
            0,
            0,
        ]);
        out.get(..Self::PAYLOAD_LEN)
    }

    /// Decodes a payload into a status frame.
    #[must_use]
    pub fn decode(payload: &[u8]) -> Option<Self> {
        if payload.len() != Self::PAYLOAD_LEN {
            return None;
        }
        let (head, tail) = payload.split_first_chunk::<5>()?;
        let (flags, _) = tail.split_first_chunk::<3>()?;
        Some(Self {
            en_3r6: head[0],
            en_36r5: head[1],
            pwm_duty: u16::from_le_bytes([head[2], head[3]]),
            gate_mask: head[4],
            tiny_all_off: flags[0] & 1 != 0,
            emergency_gate_off: flags[0] & 2 != 0,
            active_balancer_on: flags[0] & 4 != 0,
            en_all: flags[0] & 8 != 0,
            cellagent_alive: flags[0] & 16 != 0,
        })
    }
}

/// Decoded [`Kind::SetBleed`](crate::Kind::SetBleed) payload.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BleedMasks {
    /// Leg-A (2.0 ohm) enable mask, bit x = cell x+1.
    pub en_3r6: u8,
    /// Leg-B (7.2 ohm) enable mask, bit x = cell x+1.
    pub en_36r5: u8,
}

/// Decodes a [`Kind::SetBleed`](crate::Kind::SetBleed) payload.
#[must_use]
pub fn decode_bleed(payload: &[u8]) -> Option<BleedMasks> {
    let (masks, _) = payload.split_first_chunk::<2>()?;
    Some(BleedMasks {
        en_3r6: masks[0],
        en_36r5: masks[1],
    })
}

/// Decodes a [`Kind::SetBleedPwm`](crate::Kind::SetBleedPwm) payload into a
/// duty in 1/65536 units.
#[must_use]
pub fn decode_pwm(payload: &[u8]) -> Option<u16> {
    let (bytes, _) = payload.split_first_chunk::<2>()?;
    Some(u16::from_le_bytes(*bytes))
}

/// `SetPower` flag: `ACTIVE_BALANCER_ON` (U103 P04).
pub const POWER_ACTIVE_BALANCER: u8 = 1 << 0;
/// `SetPower` flag: `EN_ALL` (U103 P05).
pub const POWER_EN_ALL: u8 = 1 << 1;

#[cfg(test)]
mod tests {
    use super::{
        BalancerStatus, BleedMasks, RAIL_ORDER, RAILS, RailSnapshot, Snapshot, TEMP_INVALID,
        TEMP_ORDER, TEMPS, TempSnapshot, decode_bleed, decode_pwm, decode_temps, encode_temps,
    };

    #[test]
    fn snapshot_roundtrips() {
        let snap = Snapshot {
            seq: 0x7F,
            codes: [0x0012_3456, -1, i32::MIN, i32::MAX],
        };
        let mut buf = [0u8; Snapshot::PAYLOAD_LEN + 4];
        let payload = snap.encode(&mut buf).expect("fits");
        assert_eq!(payload.len(), Snapshot::PAYLOAD_LEN);
        assert_eq!(Snapshot::decode(payload), Some(snap));
    }

    #[test]
    fn snapshot_rejects_wrong_length() {
        assert!(Snapshot::decode(&[0; 3]).is_none());
        let mut buf = [0u8; 4];
        assert!(
            Snapshot {
                seq: 0,
                codes: [0; 4]
            }
            .encode(&mut buf)
            .is_none()
        );
    }

    #[test]
    fn rails_roundtrip_and_order_documented() {
        assert_eq!(RAIL_ORDER.len(), RAILS);
        assert_eq!(RAIL_ORDER[0], "VBAT_A");
        let snap = RailSnapshot {
            codes: [1, 2, 3, 4, 5, 6, 1023, 0],
        };
        let mut buf = [0u8; RailSnapshot::PAYLOAD_LEN + 2];
        let payload = snap.encode(&mut buf).expect("fits");
        assert_eq!(RailSnapshot::decode(payload), Some(snap));
    }

    #[test]
    fn rails_rejects_wrong_length() {
        assert!(RailSnapshot::decode(&[0x11, 0x22]).is_none());
        assert!(
            RailSnapshot { codes: [0; RAILS] }
                .encode(&mut [0u8; 4])
                .is_none()
        );
    }

    #[test]
    fn temps_roundtrip_and_order_documented() {
        assert_eq!(TEMP_ORDER.len(), TEMPS);
        let temps: TempSnapshot = [2500, -1010, TEMP_INVALID];
        let mut buf = [0u8; TEMPS * 2 + 1];
        let payload = encode_temps(&temps, &mut buf).expect("fits");
        assert_eq!(decode_temps(payload), Some(temps));
    }

    #[test]
    fn balancer_status_roundtrips_all_flags() {
        let status = BalancerStatus {
            en_3r6: 0x0F,
            en_36r5: 0x05,
            pwm_duty: 0xBEEF,
            gate_mask: 0x03,
            tiny_all_off: true,
            emergency_gate_off: true,
            active_balancer_on: false,
            en_all: true,
            cellagent_alive: true,
        };
        let mut buf = [0u8; BalancerStatus::PAYLOAD_LEN + 1];
        let payload = status.encode(&mut buf).expect("fits");
        assert_eq!(BalancerStatus::decode(payload), Some(status));
    }

    #[test]
    fn bleed_and_pwm_decode() {
        assert_eq!(
            decode_bleed(&[0x0F, 0x01]),
            Some(BleedMasks {
                en_3r6: 0x0F,
                en_36r5: 0x01
            })
        );
        assert!(decode_bleed(&[0x01]).is_none());
        assert_eq!(decode_pwm(&[0xEF, 0xBE]), Some(0xBEEF));
        assert!(decode_pwm(&[0xEF]).is_none());
    }

    #[test]
    fn power_flag_bits_are_distinct() {
        assert_eq!(super::POWER_ACTIVE_BALANCER, 0x01);
        assert_eq!(super::POWER_EN_ALL, 0x02);
    }
}
