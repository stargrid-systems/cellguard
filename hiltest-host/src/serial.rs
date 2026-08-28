//! Serial line I/O with the input-buffer flush the bring-up sessions taught.

use std::io::{self, Read, Write};
use std::time::{Duration, Instant};

use serialport::{ClearBuffer, SerialPort};

/// Poll granularity of the blocking byte reads.
const READ_TIMEOUT: Duration = Duration::from_millis(100);

/// Line-oriented reader/writer over one serial port.
///
/// Generic over the byte stream so tests can substitute a fake port. The
/// default is a real serial port.
pub struct Lines<P = Box<dyn SerialPort>> {
    port: P,
    pending: Vec<u8>,
}

impl Lines {
    /// Opens `path` at `baud` 8N1 and clears stale input, so minutes of
    /// buffered boot spam cannot poison the first exchange.
    ///
    /// # Errors
    ///
    /// Returns an error when the port cannot be opened or cleared.
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
}

impl<P: Read + Write> Lines<P> {
    /// Sends one LF-terminated line.
    ///
    /// # Errors
    ///
    /// Returns an error when the write fails.
    pub fn send(&mut self, line: &str) -> io::Result<()> {
        self.port.write_all(line.as_bytes())?;
        self.port.write_all(b"\n")?;
        self.port.flush()
    }

    /// Reads the next line, waiting at most until `deadline`. Returns
    /// [`None`] once the deadline passes. Invalid UTF-8 (opto glitch bytes)
    /// is replaced, a trailing CR is stripped.
    ///
    /// # Errors
    ///
    /// Returns an error when a read fails with anything but a timeout.
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

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;

    use super::*;

    /// Byte stream that serves scripted read chunks, then times out forever.
    struct FakePort {
        chunks: VecDeque<Vec<u8>>,
    }

    impl FakePort {
        fn new<const N: usize>(chunks: [&[u8]; N]) -> Lines<Self> {
            Lines {
                port: Self {
                    chunks: chunks.iter().map(|c| c.to_vec()).collect(),
                },
                pending: Vec::new(),
            }
        }
    }

    impl Read for FakePort {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            let Some(chunk) = self.chunks.pop_front() else {
                return Err(io::ErrorKind::TimedOut.into());
            };
            buf[..chunk.len()].copy_from_slice(&chunk);
            Ok(chunk.len())
        }
    }

    impl Write for FakePort {
        fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
            Ok(buf.len())
        }

        fn flush(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    fn generous_deadline() -> Instant {
        Instant::now() + Duration::from_secs(5)
    }

    #[test]
    fn next_line_frames_lines_split_across_chunks() {
        let mut lines = FakePort::new([b"ab", b"c\r\nd", b"ef\n"]);
        let first = lines.next_line(generous_deadline()).unwrap();
        assert_eq!(first.as_deref(), Some("abc"));
        let second = lines.next_line(generous_deadline()).unwrap();
        assert_eq!(second.as_deref(), Some("def"));
    }

    #[test]
    fn next_line_returns_none_at_the_deadline_and_keeps_the_partial() {
        let mut lines = FakePort::new([b"par"]);
        let deadline = Instant::now() + Duration::from_millis(20);
        let line = lines.next_line(deadline).unwrap();
        assert_eq!(line, None);
        assert_eq!(lines.pending, b"par");
    }

    #[test]
    fn next_line_replaces_invalid_utf8() {
        let mut lines = FakePort::new([b"a\xFFb\n"]);
        let line = lines.next_line(generous_deadline()).unwrap();
        assert_eq!(line.as_deref(), Some("a\u{FFFD}b"));
    }
}
