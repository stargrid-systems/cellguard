//! The cellagent runtime.
//!
//! [`CellagentRuntime`] decodes incoming COBS frames, dispatches requests to
//! the cellagent hardware, and writes encoded responses back to the bus.

use cellguard_panic::{PanicRecord, RECORD_LEN};
use cellguard_protocol::{Decoder, Kind, Packet, encode_frame};
use embedded_io::Write;

use crate::hw::{GateControl, TempSensor};

/// Size of the receive buffer for COBS decoding.
const RX_BUF_SIZE: usize = 64;

/// Maximum response payload: a panic record (the temperature reading is 2).
const MAX_RESPONSE_PAYLOAD: usize = RECORD_LEN;

/// Maximum raw response frame: header plus payload plus payload CRC.
const MAX_RESPONSE_RAW: usize =
    cellguard_protocol::HEADER_LEN + MAX_RESPONSE_PAYLOAD + cellguard_protocol::PAYLOAD_CRC_LEN;

/// Maximum COBS-encoded response frame.
const MAX_RESPONSE_WIRE: usize = cellguard_protocol::max_encoded_len(MAX_RESPONSE_RAW);

/// The cellagent runtime.
///
/// Wraps a [`Decoder`] and dispatches incoming packets to the cellagent
/// hardware. Construct one with [`CellagentRuntime::new`], then feed received
/// bus bytes one at a time through [`CellagentRuntime::service`].
pub struct CellagentRuntime {
    decoder: Decoder,
    node_id: u8,
    rx_buf: [u8; RX_BUF_SIZE],
    /// Cached last panic record, reported on `PanicProbe`.
    panic_record: Option<PanicRecord>,
}

impl CellagentRuntime {
    /// Creates a runtime for the given `node_id`.
    #[must_use]
    pub const fn new(node_id: u8) -> Self {
        Self {
            decoder: Decoder::new(),
            node_id,
            rx_buf: [0; RX_BUF_SIZE],
            panic_record: None,
        }
    }

    /// Caches the last panic record so a later `PanicProbe` reports it. Call
    /// this once at boot after reading the slot from EEPROM.
    pub const fn set_panic_record(&mut self, record: Option<PanicRecord>) {
        self.panic_record = record;
    }

    /// Feeds one received byte.
    ///
    /// When a complete packet is decoded, handles it and writes any response to
    /// `out`. Returns the number of bytes written, or 0 if no response was
    /// produced (incomplete frame, wrong node, or a decode error).
    pub fn service<G, T, W>(&mut self, byte: u8, gates: &mut G, temp: &mut T, out: &mut W) -> usize
    where
        G: GateControl,
        T: TempSensor,
        W: Write,
    {
        let Ok(Some(frame_len)) = self.decoder.feed(byte, &mut self.rx_buf) else {
            return 0;
        };
        let Some(frame) = self.rx_buf.get(..frame_len) else {
            return 0;
        };
        let Ok(packet) = Packet::parse(frame) else {
            return 0;
        };
        if packet.id != self.node_id {
            return 0;
        }

        match packet.kind {
            Kind::ReadTemperature => {
                let centi = temp.read_centi_celsius();
                let payload = centi.to_le_bytes();
                self.write_response(Kind::Temperature, &payload, out)
            }
            Kind::SetBalancer => match packet.payload {
                &[mask] => {
                    gates.set_gates(mask);
                    self.write_response(Kind::Ack, &[], out)
                }
                _ => self.write_response(Kind::Nack, &[], out),
            },
            Kind::PanicProbe => match self.panic_record {
                Some(record) => self.write_response(Kind::PanicStatus, &record.serialize(), out),
                None => self.write_response(Kind::PanicStatus, &[], out),
            },
            _ => self.write_response(Kind::Nack, &[], out),
        }
    }

    /// Builds and writes a response packet COBS-encoded onto `out`.
    fn write_response<W: Write>(&self, kind: Kind, payload: &[u8], out: &mut W) -> usize {
        let mut raw = [0u8; MAX_RESPONSE_RAW];
        let Ok(raw_len) = Packet::write(self.node_id, kind, payload, &mut raw) else {
            return 0;
        };
        let Some(raw_slice) = raw.get(..raw_len) else {
            return 0;
        };

        let mut wire = [0u8; MAX_RESPONSE_WIRE];
        let Some(wire_len) = encode_frame(raw_slice, &mut wire) else {
            return 0;
        };
        let Some(wire_slice) = wire.get(..wire_len) else {
            return 0;
        };

        if out.write_all(wire_slice).is_err() {
            return 0;
        }
        wire_len
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
            runtime.service(byte, &mut gates, &mut temp, &mut writer);
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
            runtime.service(byte, &mut gates, &mut temp, &mut writer);
        }

        let (kind, payload) = decode_response(&writer.buf);
        assert_eq!(kind, Kind::Temperature);
        assert_eq!(payload, TEMP_CENTI.to_le_bytes());
    }

    fn sample_record() -> cellguard_panic::PanicRecord {
        let mut file = [0u8; cellguard_panic::FILE_CAP];
        file[..3].copy_from_slice(b"app");
        cellguard_panic::PanicRecord {
            reset_flags: 0x10,
            consecutive_panics: 1,
            file,
            file_len: 3,
            line: 7,
            col: 0,
        }
    }

    #[test]
    fn panic_probe_returns_cached_record() {
        let mut runtime = CellagentRuntime::new(NODE);
        runtime.set_panic_record(Some(sample_record()));
        let mut gates = MockGates { mask: 0 };
        let mut temp = MockTemp { value: 0 };
        let mut writer = VecWriter { buf: Vec::new() };

        let wire = encode_request(Kind::PanicProbe, &[]);
        for &byte in &wire {
            runtime.service(byte, &mut gates, &mut temp, &mut writer);
        }

        let (kind, payload) = decode_response(&writer.buf);
        assert_eq!(kind, Kind::PanicStatus);
        assert_eq!(payload.len(), cellguard_panic::RECORD_LEN);
        let bytes: &[u8; cellguard_panic::RECORD_LEN] = payload.as_slice().try_into().unwrap();
        assert_eq!(
            cellguard_panic::PanicRecord::parse(bytes).unwrap(),
            sample_record()
        );
    }

    #[test]
    fn panic_probe_without_record_is_empty() {
        let mut runtime = CellagentRuntime::new(NODE);
        let mut gates = MockGates { mask: 0 };
        let mut temp = MockTemp { value: 0 };
        let mut writer = VecWriter { buf: Vec::new() };

        let wire = encode_request(Kind::PanicProbe, &[]);
        for &byte in &wire {
            runtime.service(byte, &mut gates, &mut temp, &mut writer);
        }

        let (kind, payload) = decode_response(&writer.buf);
        assert_eq!(kind, Kind::PanicStatus);
        assert!(payload.is_empty());
    }
}
