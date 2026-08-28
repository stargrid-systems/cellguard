//! Command/response session with the test firmware.

use std::io;
use std::time::{Duration, Instant};

use hiltest_protocol::{Command, Event, Outcome, TestId};

use crate::report::Verdict;
use crate::serial::Lines;

/// Payload for the echo tests: letters, digits, and punctuation, no
/// whitespace.
const ECHO_PAYLOAD: &str = "hil-echo-0123456789-abcxyz!";

const BANNER_TIMEOUT: Duration = Duration::from_secs(6);
const PING_TIMEOUT: Duration = Duration::from_secs(2);
const PING_RETRIES: u32 = 3;

/// One serial session with a board running the test firmware.
pub struct Session {
    lines: Lines,
    ping_seq: u32,
}

impl Session {
    /// Opens the port. The input buffer is cleared by [`Lines::open`].
    ///
    /// # Errors
    ///
    /// Returns an error when the port cannot be opened.
    pub fn open(port: &str, baud: u32) -> io::Result<Self> {
        Ok(Self {
            lines: Lines::open(port, baud)?,
            ping_seq: 0,
        })
    }

    /// Verifies the link. With `expect_banner` (right after a flash) it
    /// first waits for the firmware's boot banner and ready line.
    ///
    /// # Errors
    ///
    /// Returns an error when the port fails or the firmware does not answer.
    pub fn wait_ready(&mut self, expect_banner: bool) -> io::Result<()> {
        if expect_banner {
            let deadline = Instant::now() + BANNER_TIMEOUT;
            loop {
                let Some(line) = self.lines.next_line(deadline)? else {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        "no boot banner from the test firmware",
                    ));
                };
                note(&line);
                if matches!(Event::parse(&line), Some(Event::Ready)) {
                    break;
                }
            }
        }
        self.ping()
    }

    fn ping(&mut self) -> io::Result<()> {
        for _ in 0..PING_RETRIES {
            self.ping_seq += 1;
            let n = self.ping_seq;
            self.lines.send(&Command::Ping(n).to_string())?;
            let deadline = Instant::now() + PING_TIMEOUT;
            while let Some(line) = self.lines.next_line(deadline)? {
                note(&line);
                if matches!(Event::parse(&line), Some(Event::Pong(m)) if m == n) {
                    return Ok(());
                }
            }
        }
        Err(io::Error::new(
            io::ErrorKind::TimedOut,
            "no pong from the test firmware",
        ))
    }

    /// Runs one test and waits for its result line. Reboot banners mid-test
    /// are tolerated: the deferred result arrives right after the banner.
    ///
    /// # Errors
    ///
    /// Returns an error when the port fails. A missing result line is a
    /// [`Verdict::Timeout`], not an error.
    pub fn run_test(&mut self, id: TestId, timeout: Duration) -> io::Result<Verdict> {
        self.lines.send(&Command::Run(id.name()).to_string())?;
        let deadline = Instant::now() + timeout;
        let mut echo_sent = false;
        let mut echo_ok = true;
        loop {
            let Some(line) = self.lines.next_line(deadline)? else {
                return Ok(Verdict::Timeout);
            };
            note(&line);
            match Event::parse(&line) {
                Some(Event::Log { body }) => {
                    // The echo tests prompt for their payload line.
                    if !echo_sent && body.strip_prefix(id.name()) == Some(" send") {
                        self.lines.send(ECHO_PAYLOAD)?;
                        echo_sent = true;
                    }
                }
                Some(Event::Echo { payload }) => {
                    if payload != ECHO_PAYLOAD {
                        echo_ok = false;
                    }
                }
                Some(Event::Result {
                    id: result_id,
                    outcome,
                    detail,
                }) if result_id == id.name() => {
                    if outcome == Outcome::Pass && !echo_ok {
                        return Ok(Verdict::Fail(Some("echo-mismatch".to_owned())));
                    }
                    let detail = detail.map(str::to_owned);
                    return Ok(match outcome {
                        Outcome::Pass => Verdict::Pass,
                        Outcome::Fail => Verdict::Fail(detail),
                        Outcome::Skip => Verdict::Skip(detail),
                    });
                }
                Some(Event::Err { reason }) => {
                    return Ok(Verdict::Fail(Some(format!("firmware: {reason}"))));
                }
                // Everything else (ack, banners, ready, noise) was printed
                // above and needs no reaction.
                _ => {}
            }
        }
    }
}

/// Prints a board line as indented diagnostics.
fn note(line: &str) {
    println!("  {line}");
}
