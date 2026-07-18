//! Serial-port transport with COBS-framed packet exchange.
//!
//! [`Transport`] wraps a [`serialport::SerialPort`] and the protocol's
//! streaming COBS [`Decoder`]. [`Transport::exchange`] sends one command packet
//! and blocks until a complete response frame is decoded.

use std::io::{self, Read, Write};
use std::time::Duration;

use cellguard_protocol::{
    DecodeError, Decoder, Kind, Packet, encode_frame, max_encoded_len,
};
use serialport::SerialPort;

/// Maximum decoded frame we can receive. The biggest response payload is a
/// `PersistentState` (28 bytes) or a `PanicRecord` (64 bytes); either fits well
/// within this bound with room for future growth.
const RX_BUF: usize = 256;

/// Scratch for building the outgoing pre-COBS frame.
const TX_RAW: usize = 256;

/// Worst-case COBS-encoded outgoing frame.
const TX_WIRE: usize = max_encoded_len(TX_RAW);

/// Reads/writes `CellGuard` bus packets over a serial port.
pub struct Transport {
    port: Box<dyn SerialPort>,
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
        Ok(Self {
            port,
            decoder: Decoder::new(),
            rx: [0u8; RX_BUF],
        })
    }

    /// Sends a command packet addressed to `id` and blocks for the response.
    ///
    /// Returns the parsed response packet. The response `id` is not checked:
    /// the device always replies with the sender's `id`, and on a point-to-point
    /// field-bus link there is exactly one responder.
    ///
    /// # Errors
    ///
    /// Returns an error on I/O failure, COBS decode failure, or if the response
    /// packet fails its CRC or carries an unknown kind.
    pub fn exchange(&mut self, id: u8, kind: Kind, payload: &[u8]) -> io::Result<Reply> {
        self.send(id, kind, payload)?;
        self.recv()
    }

    fn send(&mut self, id: u8, kind: Kind, payload: &[u8]) -> io::Result<()> {
        let mut raw = vec![0u8; TX_RAW];
        let raw_len = Packet::write(id, kind, payload, &mut raw)
            .map_err(|e| io::Error::other(format!("packet write failed: {e}")))?;
        let mut wire = vec![0u8; TX_WIRE];
        let wire_len = encode_frame(&raw[..raw_len], &mut wire)
            .ok_or_else(|| io::Error::other("COBS encode failed: output too small"))?;
        self.port
            .write_all(&wire[..wire_len])?;
        self.port.flush()
    }

    fn recv(&mut self) -> io::Result<Reply> {
        let mut byte = [0u8; 1];
        loop {
            match self.port.read(&mut byte) {
                Ok(0) => return Err(io::Error::new(io::ErrorKind::TimedOut, "serial read timed out")),
                Ok(_) => {}
                Err(ref e) if e.kind() == io::ErrorKind::TimedOut => {
                    return Err(io::Error::new(io::ErrorKind::TimedOut, "no response within timeout"));
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
                    return Err(io::Error::other(
                        "response frame too large for RX buffer",
                    ));
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
pub struct Reply {
    /// The response kind.
    pub kind: Kind,
    /// The response payload bytes.
    pub payload: Vec<u8>,
}
