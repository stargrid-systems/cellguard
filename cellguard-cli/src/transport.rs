//! Serial-port transport with COBS-framed packet exchange.
//!
//! [`Transport::exchange`] sends one command packet and blocks until a
//! complete response frame is decoded.

use std::io::{self, Read, Write};
use std::time::Duration;

use cellguard_protocol::{DecodeError, Decoder, Kind, Packet, encode_frame, max_encoded_len};
use serialport::SerialPort;

/// The largest response payload is a `PersistentState` (28 B) or a
/// `PanicRecord` (64 B), so 256 leaves headroom.
const RX_BUF: usize = 256;

const TX_RAW: usize = 256;

const TX_WIRE: usize = max_encoded_len(TX_RAW);

/// Reads/writes `CellGuard` bus packets over a byte stream.
///
/// The stream is a serial port in production, opened with [`Transport::open`].
/// Any other `Read + Write` type works through [`Transport::new`].
pub struct Transport<P = Box<dyn SerialPort>> {
    port: P,
    decoder: Decoder,
    rx: [u8; RX_BUF],
}

impl Transport {
    /// Opens the serial port at `path` with `baud` 8N1 and a 2 s read timeout.
    ///
    /// # Errors
    ///
    /// Returns an error if the port cannot be opened.
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(Duration::from_secs(2))
            .open()
            .map_err(io::Error::other)?;
        Ok(Self::new(port))
    }
}

impl<P: Read + Write> Transport<P> {
    /// Wraps an already-open byte stream.
    ///
    /// This is the test seam: any `Read + Write` type can stand in for the
    /// serial port.
    ///
    /// # Examples
    ///
    /// ```
    /// use std::io::Cursor;
    ///
    /// use cellguard_cli::transport::Transport;
    ///
    /// let _transport = Transport::new(Cursor::new(Vec::new()));
    /// ```
    #[must_use]
    pub const fn new(port: P) -> Self {
        Self {
            port,
            decoder: Decoder::new(),
            rx: [0u8; RX_BUF],
        }
    }

    /// Sends a command packet addressed to `id` and blocks for the response.
    ///
    /// The response `id` is not checked: the point-to-point link has exactly
    /// one responder.
    ///
    /// # Errors
    ///
    /// Returns an error if the write fails, the read times out, or the
    /// response frame does not decode into a valid packet.
    pub fn exchange(&mut self, id: u8, kind: Kind, payload: &[u8]) -> io::Result<Reply> {
        self.send(id, kind, payload)?;
        self.recv()
    }

    fn send(&mut self, id: u8, kind: Kind, payload: &[u8]) -> io::Result<()> {
        let mut raw = vec![0u8; TX_RAW];
        let raw_len = Packet::write(id, kind, payload, &mut raw)
            .map_err(|e| io::Error::other(format!("packet write failed: {e}")))?;
        let raw_frame = raw
            .get(..raw_len)
            .ok_or_else(|| io::Error::other("internal: packet longer than TX buffer"))?;
        let mut wire = vec![0u8; TX_WIRE];
        let wire_len = encode_frame(raw_frame, &mut wire)
            .ok_or_else(|| io::Error::other("COBS encode failed: output too small"))?;
        let wire_frame = wire
            .get(..wire_len)
            .ok_or_else(|| io::Error::other("internal: encoded frame longer than wire buffer"))?;
        self.port.write_all(wire_frame)?;
        self.port.flush()
    }

    fn recv(&mut self) -> io::Result<Reply> {
        let mut byte = [0u8; 1];
        loop {
            match self.port.read(&mut byte) {
                Ok(0) => {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "serial read timed out",
                    ));
                }
                Ok(_) => {}
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "no response within timeout",
                    ));
                }
                Err(e) => return Err(e),
            }
            match self.decoder.feed(byte[0], &mut self.rx) {
                Ok(Some(len)) => {
                    let frame = self.rx.get(..len).ok_or_else(|| {
                        io::Error::other("internal: decoded frame longer than RX buffer")
                    })?;
                    let packet = Packet::parse(frame)
                        .map_err(|e| io::Error::other(format!("response parse failed: {e}")))?;
                    let payload = packet.payload.to_vec();
                    return Ok(Reply {
                        kind: packet.kind,
                        payload,
                    });
                }
                Ok(None) => {}
                Err(DecodeError::BufferTooSmall) => {
                    return Err(io::Error::other("response frame too large for RX buffer"));
                }
                Err(DecodeError::InvalidFrame) => {
                    return Err(io::Error::other("malformed COBS frame on bus"));
                }
                Err(_) => {
                    return Err(io::Error::other("unknown COBS decode error"));
                }
            }
        }
    }
}

/// A decoded response packet with an owned payload copy.
#[derive(Debug)]
pub struct Reply {
    /// Message kind.
    pub kind: Kind,
    /// Payload bytes.
    pub payload: Vec<u8>,
}

#[cfg(test)]
mod tests {
    use std::io::{self, Cursor, Read, Write};

    use cellguard_protocol::{Decoder, Kind, Packet, encode_frame, max_encoded_len};

    use super::Transport;

    struct FakePort {
        rx: Cursor<Vec<u8>>,
        tx: Vec<u8>,
    }

    impl FakePort {
        fn new(rx: Vec<u8>) -> Self {
            Self {
                rx: Cursor::new(rx),
                tx: Vec::new(),
            }
        }
    }

    impl Read for FakePort {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            self.rx.read(buf)
        }
    }

    impl Write for FakePort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            self.tx.write(buf)
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    fn encode_reply(id: u8, kind: Kind, payload: &[u8]) -> Vec<u8> {
        let mut raw = vec![0u8; 256];
        let raw_len = Packet::write(id, kind, payload, &mut raw).unwrap();
        let mut wire = vec![0u8; max_encoded_len(256)];
        let wire_len = encode_frame(&raw[..raw_len], &mut wire).unwrap();
        wire[..wire_len].to_vec()
    }

    fn decode_frame(wire: &[u8]) -> Vec<u8> {
        let mut decoder = Decoder::new();
        let mut buf = [0u8; 512];
        for &byte in wire {
            if let Some(len) = decoder.feed(byte, &mut buf).unwrap() {
                return buf[..len].to_vec();
            }
        }
        panic!("no complete frame in wire bytes");
    }

    #[test]
    fn send_produces_a_decodable_frame() {
        let mut transport = Transport::new(FakePort::new(Vec::new()));
        transport.send(5, Kind::BootProbe, &[1, 2, 3]).unwrap();

        let frame = decode_frame(&transport.port.tx);
        let packet = Packet::parse(&frame).unwrap();
        assert_eq!(packet.id, 5);
        assert_eq!(packet.kind, Kind::BootProbe);
        assert_eq!(packet.payload, [1, 2, 3]);
    }

    #[test]
    fn recv_decodes_a_reply() {
        let wire = encode_reply(1, Kind::BootAck, &[4, 0, 0, 0]);
        let mut transport = Transport::new(FakePort::new(wire));

        let reply = transport.recv().unwrap();
        assert_eq!(reply.kind, Kind::BootAck);
        assert_eq!(reply.payload, [4, 0, 0, 0]);
    }

    #[test]
    fn recv_times_out_on_eof() {
        let mut transport = Transport::new(FakePort::new(Vec::new()));
        let err = transport.recv().unwrap_err();
        assert_eq!(err.kind(), io::ErrorKind::TimedOut);
    }

    #[test]
    fn recv_rejects_a_corrupt_frame() {
        let mut wire = encode_reply(1, Kind::BootAck, &[4, 0, 0, 0]);
        let last_payload_index = wire.len() - 4;
        wire[last_payload_index] ^= 0x55;
        let mut transport = Transport::new(FakePort::new(wire));
        assert!(transport.recv().is_err());
    }

    #[test]
    fn exchange_round_trips() {
        let wire = encode_reply(2, Kind::Ack, &[]);
        let mut transport = Transport::new(FakePort::new(wire));

        let reply = transport.exchange(2, Kind::SetPower, &[1]).unwrap();
        assert_eq!(reply.kind, Kind::Ack);
        assert!(reply.payload.is_empty());

        let frame = decode_frame(&transport.port.tx);
        let packet = Packet::parse(&frame).unwrap();
        assert_eq!(packet.id, 2);
        assert_eq!(packet.kind, Kind::SetPower);
        assert_eq!(packet.payload, [1]);
    }
}
