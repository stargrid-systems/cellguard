//! The cellagent runtime.
//!
//! [`CellagentRuntime`] decodes incoming COBS frames, dispatches requests to
//! the cellagent hardware, and writes encoded responses back to the bus.
//!
//! # Gate safety timeout
//!
//! The gates are a safety actuator. If the host stops refreshing
//! `SetBalancer` (crash, unplugged cable), the gates must not hold their last
//! state forever. The runtime therefore arms a refresh timeout on every
//! `SetBalancer` and drives the gates to the safe state ([`SAFE_GATE_MASK`])
//! once [`CellagentRuntime::check_timeout`] sees the deadline pass. The
//! caller owns the time base and passes a free-running tick value to both
//! [`CellagentRuntime::service`] and `check_timeout`, so the timeout is exact
//! on hardware and deterministic in tests.

use cellguard_protocol::{Decoder, Kind, Packet, encode_frame};
use embedded_io::Write;

use crate::hw::{GateControl, TempSensor};

/// Size of the receive buffer for COBS decoding.
const RX_BUF_SIZE: usize = 64;

/// Maximum response payload: the temperature reading in centi-degrees Celsius.
const MAX_RESPONSE_PAYLOAD: usize = 2;

/// Maximum raw response frame: header plus payload plus payload CRC.
const MAX_RESPONSE_RAW: usize =
    cellguard_protocol::HEADER_LEN + MAX_RESPONSE_PAYLOAD + cellguard_protocol::PAYLOAD_CRC_LEN;

/// Maximum COBS-encoded response frame.
const MAX_RESPONSE_WIRE: usize = cellguard_protocol::max_encoded_len(MAX_RESPONSE_RAW);

/// The gate mask driven when the refresh timeout fires or no command has
/// arrived yet: both gate lines low and `ALL_OFF` asserted, which the
/// balancing hardware decodes as "everything off".
pub const SAFE_GATE_MASK: u8 = 0x04;

/// Default refresh window in caller ticks (roughly 2 s at 1.024 kHz).
pub const DEFAULT_GATE_TIMEOUT_TICKS: u16 = 2048;

/// The cellagent runtime.
///
/// Wraps a [`Decoder`] and dispatches incoming packets to the cellagent
/// hardware. Construct one with [`CellagentRuntime::new`], then feed received
/// bus bytes one at a time through [`CellagentRuntime::service`].
pub struct CellagentRuntime {
    decoder: Decoder,
    node_id: u8,
    rx_buf: [u8; RX_BUF_SIZE],
    gate_mask: u8,
    gate_timeout: u16,
    last_refresh: u16,
    armed: bool,
}

impl CellagentRuntime {
    /// Creates a runtime for the given `node_id` with the default gate
    /// refresh timeout.
    #[must_use]
    pub const fn new(node_id: u8) -> Self {
        Self::with_gate_timeout(node_id, DEFAULT_GATE_TIMEOUT_TICKS)
    }

    /// Creates a runtime whose gates trip to [`SAFE_GATE_MASK`] when no
    /// `SetBalancer` arrives within `gate_timeout` ticks.
    #[must_use]
    pub const fn with_gate_timeout(node_id: u8, gate_timeout: u16) -> Self {
        Self {
            decoder: Decoder::new(),
            node_id,
            rx_buf: [0; RX_BUF_SIZE],
            gate_mask: SAFE_GATE_MASK,
            gate_timeout,
            last_refresh: 0,
            armed: false,
        }
    }

    /// The last commanded gate mask. Reports the safe mask before any command
    /// and after a timeout trip, so the echo always reflects reality.
    #[must_use]
    pub const fn gate_mask(&self) -> u8 {
        self.gate_mask
    }

    /// Feeds one received byte.
    ///
    /// `tick` is the caller's free-running time base (see the crate docs).
    /// When a complete packet is decoded, handles it and writes any response
    /// to `out`. No response is produced (and nothing is written) for an
    /// incomplete frame, a frame addressed to another node, or a decode
    /// error.
    pub fn service<G, T, W>(
        &mut self,
        byte: u8,
        tick: u16,
        gates: &mut G,
        temp: &mut T,
        out: &mut W,
    ) where
        G: GateControl,
        T: TempSensor,
        W: Write,
    {
        let Ok(Some(frame_len)) = self.decoder.feed(byte, &mut self.rx_buf) else {
            return;
        };
        let Some(frame) = self.rx_buf.get(..frame_len) else {
            return;
        };
        let Ok(packet) = Packet::parse(frame) else {
            return;
        };
        if packet.id != self.node_id {
            return;
        }

        match packet.kind {
            Kind::ReadTemperature => {
                let centi = temp.read_centi_celsius();
                let payload = centi.to_le_bytes();
                self.write_response(Kind::Temperature, &payload, out);
            }
            Kind::SetBalancer => match packet.payload {
                &[mask] => {
                    gates.set_gates(mask);
                    self.gate_mask = mask;
                    self.last_refresh = tick;
                    self.armed = true;
                    self.write_response(Kind::Ack, &[], out);
                }
                _ => self.write_response(Kind::Nack, &[], out),
            },
            Kind::ReadBalancerGateState => {
                let payload = [self.gate_mask];
                self.write_response(Kind::BalancerGateState, &payload, out);
            }
            Kind::PanicProbe => self.write_response(Kind::PanicStatus, &[], out),
            _ => self.write_response(Kind::Nack, &[], out),
        }
    }

    /// Drives the gates to the safe state when the refresh window has
    /// elapsed. Call every loop iteration with the current tick. The check is
    /// inert until the first `SetBalancer` arms it: a link that is silent
    /// from power-up never commanded gates on, so there is nothing to trip.
    pub fn check_timeout<G: GateControl>(&mut self, tick: u16, gates: &mut G) {
        if self.armed && tick.wrapping_sub(self.last_refresh) > self.gate_timeout {
            gates.set_gates(SAFE_GATE_MASK);
            self.gate_mask = SAFE_GATE_MASK;
            self.armed = false;
        }
    }

    /// Builds and writes a response packet COBS-encoded onto `out`.
    fn write_response<W: Write>(&self, kind: Kind, payload: &[u8], out: &mut W) {
        let mut raw = [0u8; MAX_RESPONSE_RAW];
        let Ok(raw_len) = Packet::write(self.node_id, kind, payload, &mut raw) else {
            return;
        };
        let Some(raw_slice) = raw.get(..raw_len) else {
            return;
        };

        let mut wire = [0u8; MAX_RESPONSE_WIRE];
        let Some(wire_len) = encode_frame(raw_slice, &mut wire) else {
            return;
        };
        let Some(wire_slice) = wire.get(..wire_len) else {
            return;
        };

        let _ = out.write_all(wire_slice);
    }
}

#[cfg(test)]
mod tests {
    use std::vec::Vec;

    use cellguard_protocol::{Decoder, Kind, Packet, encode_frame, max_encoded_len};

    use super::CellagentRuntime;
    use crate::hw::{GateControl, TempSensor};

    const NODE: u8 = 7;

    struct MockGates {
        mask: u8,
    }

    impl GateControl for MockGates {
        fn set_gates(&mut self, mask: u8) {
            self.mask = mask;
        }
    }

    struct MockTemp {
        value: i16,
    }

    impl TempSensor for MockTemp {
        fn read_centi_celsius(&mut self) -> i16 {
            self.value
        }
    }

    struct VecWriter {
        buf: Vec<u8>,
    }

    impl embedded_io::ErrorType for VecWriter {
        type Error = core::convert::Infallible;
    }

    impl embedded_io::Write for VecWriter {
        fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
            self.buf.extend_from_slice(buf);
            Ok(buf.len())
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }

    /// COBS-encodes a request packet addressed to [`NODE`].
    fn encode_request(kind: Kind, payload: &[u8]) -> Vec<u8> {
        let mut raw = [0u8; 32];
        let raw_len = Packet::write(NODE, kind, payload, &mut raw).expect("test: write raw packet");
        let cap = max_encoded_len(raw_len);
        let mut wire = std::vec![0u8; cap];
        let n = encode_frame(
            raw.get(..raw_len).expect("test: raw slice in bounds"),
            &mut wire,
        )
        .expect("test: encode COBS frame");
        wire.truncate(n);
        wire
    }

    /// Decodes the first packet from a COBS wire stream.
    fn decode_response(wire: &[u8]) -> (Kind, Vec<u8>) {
        let mut decoder = Decoder::new();
        let mut scratch = [0u8; 128];
        for &byte in wire {
            if let Ok(Some(n)) = decoder.feed(byte, &mut scratch)
                && let Some(frame) = scratch.get(..n)
                && let Ok(packet) = Packet::parse(frame)
            {
                return (packet.kind, packet.payload.to_vec());
            }
        }
        panic!("test: no response frame decoded");
    }

    #[test]
    fn set_balancer_produces_ack() {
        let mut runtime = CellagentRuntime::new(NODE);
        let mut gates = MockGates { mask: 0 };
        let mut temp = MockTemp { value: 0 };
        let mut writer = VecWriter { buf: Vec::new() };

        let wire = encode_request(Kind::SetBalancer, &[0x03]);
        for &byte in &wire {
            runtime.service(byte, 0, &mut gates, &mut temp, &mut writer);
        }

        assert_eq!(gates.mask, 0x03);
        let (kind, payload) = decode_response(&writer.buf);
        assert_eq!(kind, Kind::Ack);
        assert!(payload.is_empty());
    }

    #[test]
    fn read_temperature_produces_temperature() {
        const TEMP_CENTI: i16 = 2500;
        let mut runtime = CellagentRuntime::new(NODE);
        let mut gates = MockGates { mask: 0 };
        let mut temp = MockTemp { value: TEMP_CENTI };
        let mut writer = VecWriter { buf: Vec::new() };

        let wire = encode_request(Kind::ReadTemperature, &[]);
        for &byte in &wire {
            runtime.service(byte, 0, &mut gates, &mut temp, &mut writer);
        }

        let (kind, payload) = decode_response(&writer.buf);
        assert_eq!(kind, Kind::Temperature);
        assert_eq!(payload, TEMP_CENTI.to_le_bytes());
    }

    #[test]
    fn panic_probe_without_record_is_empty() {
        let mut runtime = CellagentRuntime::new(NODE);
        let mut gates = MockGates { mask: 0 };
        let mut temp = MockTemp { value: 0 };
        let mut writer = VecWriter { buf: Vec::new() };

        let wire = encode_request(Kind::PanicProbe, &[]);
        for &byte in &wire {
            runtime.service(byte, 0, &mut gates, &mut temp, &mut writer);
        }

        let (kind, payload) = decode_response(&writer.buf);
        assert_eq!(kind, Kind::PanicStatus);
        assert!(payload.is_empty());
    }

    #[test]
    fn gate_state_echoes_last_commanded_mask() {
        let mut runtime = CellagentRuntime::new(NODE);
        let mut gates = MockGates { mask: 0 };
        let mut temp = MockTemp { value: 0 };
        let mut writer = VecWriter { buf: Vec::new() };

        let wire = encode_request(Kind::ReadBalancerGateState, &[]);
        for &byte in &wire {
            runtime.service(byte, 0, &mut gates, &mut temp, &mut writer);
        }

        let (kind, payload) = decode_response(&writer.buf);
        assert_eq!(kind, Kind::BalancerGateState);
        assert_eq!(payload, &[super::SAFE_GATE_MASK]);
    }

    #[test]
    fn gate_state_reflects_a_prior_command() {
        let mut runtime = CellagentRuntime::new(NODE);
        let mut gates = MockGates { mask: 0 };
        let mut temp = MockTemp { value: 0 };
        let mut writer = VecWriter { buf: Vec::new() };

        let wire = encode_request(Kind::SetBalancer, &[0x03]);
        for &byte in &wire {
            runtime.service(byte, 0, &mut gates, &mut temp, &mut writer);
        }
        assert_eq!(runtime.gate_mask(), 0x03);

        writer.buf.clear();
        let wire = encode_request(Kind::ReadBalancerGateState, &[]);
        for &byte in &wire {
            runtime.service(byte, 0, &mut gates, &mut temp, &mut writer);
        }
        let (kind, payload) = decode_response(&writer.buf);
        assert_eq!(kind, Kind::BalancerGateState);
        assert_eq!(payload, &[0x03]);
    }

    #[test]
    fn gate_timeout_trips_to_safe_mask() {
        let mut runtime = CellagentRuntime::with_gate_timeout(NODE, 100);
        let mut gates = MockGates { mask: 0 };
        let mut temp = MockTemp { value: 0 };
        let mut writer = VecWriter { buf: Vec::new() };

        // Silence from power-up: the timeout must not trip (nothing armed).
        runtime.check_timeout(10_000, &mut gates);
        assert_eq!(gates.mask, 0);

        // Command gates on at tick 1000.
        let wire = encode_request(Kind::SetBalancer, &[0x03]);
        for &byte in &wire {
            runtime.service(byte, 1000, &mut gates, &mut temp, &mut writer);
        }
        assert_eq!(gates.mask, 0x03);

        // Inside the window: no trip.
        runtime.check_timeout(1050, &mut gates);
        assert_eq!(gates.mask, 0x03);

        // Past the window: gates trip, echo reports the safe mask, and the
        // timeout does not re-fire.
        runtime.check_timeout(1101, &mut gates);
        assert_eq!(gates.mask, super::SAFE_GATE_MASK);
        assert_eq!(runtime.gate_mask(), super::SAFE_GATE_MASK);
        gates.mask = 0x03;
        runtime.check_timeout(5000, &mut gates);
        assert_eq!(gates.mask, 0x03, "disarmed after one trip");
    }

    #[test]
    fn a_refresh_rearms_the_timeout() {
        let mut runtime = CellagentRuntime::with_gate_timeout(NODE, 100);
        let mut gates = MockGates { mask: 0 };
        let mut temp = MockTemp { value: 0 };
        let mut writer = VecWriter { buf: Vec::new() };

        let wire = encode_request(Kind::SetBalancer, &[0x01]);
        for &byte in &wire {
            runtime.service(byte, 0, &mut gates, &mut temp, &mut writer);
        }

        // Refresh at the last moment; the window slides.
        let wire = encode_request(Kind::SetBalancer, &[0x01]);
        for &byte in &wire {
            runtime.service(byte, 100, &mut gates, &mut temp, &mut writer);
        }
        runtime.check_timeout(150, &mut gates);
        assert_eq!(gates.mask, 0x01, "refresh slides the window");
    }
}
