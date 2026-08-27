//! Serial line I/O with the input-buffer flush the bring-up sessions taught.

use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use serialport::{ClearBuffer, SerialPort};

/// Poll granularity of the blocking byte reads.
const READ_TIMEOUT: Duration = Duration::from_millis(100);

/// Line-oriented reader/writer over one serial port.
pub struct Lines {
    port: Box<dyn SerialPort>,
    pending: Vec<u8>,
}

impl Lines {
    /// Opens `path` at `baud` 8N1 and clears stale input, so minutes of
    /// buffered boot spam cannot poison the first exchange.
    pub fn open(path: &str, baud: u32) -> io::Result<Self> {
        let port = serialport::new(path, baud)
            .timeout(READ_TIMEOUT)
            .open()
            .map_err(io::Error::other)?;
        port.clear(ClearBuffer::Input).map_err(io::Error::other)?;
        Ok(Self {
            port,
            pending: Vec::new(),
        })
    }

    /// Sends one LF-terminated line.
    pub fn send(&mut self, line: &str) -> io::Result<()> {
        self.port.write_all(line.as_bytes())?;
        self.port.write_all(b"\n")?;
        self.port.flush()
    }

    /// Reads the next line, waiting at most until `deadline`. Returns
    /// [`None`] once the deadline passes. Invalid UTF-8 (opto glitch bytes)
    /// is replaced, a trailing CR is stripped.
    pub fn next_line(&mut self, deadline: Instant) -> io::Result<Option<String>> {
        loop {
            if let Some(pos) = self.pending.iter().position(|&b| b == b'\n') {
                let mut raw: Vec<u8> = self.pending.drain(..=pos).collect();
                raw.pop();
                if raw.last() == Some(&b'\r') {
                    raw.pop();
                }
                return Ok(Some(String::from_utf8_lossy(&raw).into_owned()));
            }
            if Instant::now() >= deadline {
                return Ok(None);
            }
            let mut chunk = [0u8; 256];
            match self.port.read(&mut chunk) {
                Ok(n) => self
                    .pending
                    .extend_from_slice(chunk.get(..n).unwrap_or(&[])),
                Err(e)
                    if e.kind() == io::ErrorKind::TimedOut
                        || e.kind() == io::ErrorKind::Interrupted => {}
                Err(e) => return Err(e),
            }
        }
    }
}
