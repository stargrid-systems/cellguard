//! Line grammar: the sentinel, host commands, and firmware events.

use crate::test_id::Outcome;

/// Prefix of every machine-readable firmware line. A host parser drops any
/// line without it.
pub const SENTINEL: &str = "|HIL ";

/// A host-to-firmware command line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command<'a> {
    /// Link check. The firmware answers with [`Event::Pong`] echoing the
    /// number.
    Ping(u32),
    /// Ask for one [`Event::Test`] line per known test.
    List,
    /// Run the test with this wire name.
    Run(&'a str),
    /// Software-reset the device.
    Reboot,
}

impl<'a> Command<'a> {
    /// Parses one command line. Leading and trailing whitespace is ignored.
    ///
    /// # Errors
    ///
    /// Returns [`CommandError::Unknown`] for an unknown command word and
    /// [`CommandError::BadArgument`] for missing, extra, or malformed
    /// arguments.
    pub fn parse(line: &'a str) -> Result<Self, CommandError> {
        let mut words = line.split_ascii_whitespace();
        let word = words.next().ok_or(CommandError::Unknown)?;
        let command = match word {
            "PING" => {
                let n = words.next().ok_or(CommandError::BadArgument)?;
                Self::Ping(n.parse().map_err(|_| CommandError::BadArgument)?)
            }
            "LIST" => Self::List,
            "RUN" => Self::Run(words.next().ok_or(CommandError::BadArgument)?),
            "REBOOT" => Self::Reboot,
            _ => return Err(CommandError::Unknown),
        };
        if words.next().is_some() {
            return Err(CommandError::BadArgument);
        }
        Ok(command)
    }
}

impl core::fmt::Display for Command<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Ping(n) => write!(f, "PING {n}"),
            Self::List => f.write_str("LIST"),
            Self::Run(id) => write!(f, "RUN {id}"),
            Self::Reboot => f.write_str("REBOOT"),
        }
    }
}

/// Why a command line failed to parse.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CommandError {
    /// The command word is not part of the protocol.
    Unknown,
    /// An argument is missing, extra, or malformed.
    BadArgument,
}

impl CommandError {
    /// The reason token the firmware reports in an [`Event::Err`] line.
    #[must_use]
    pub const fn token(self) -> &'static str {
        match self {
            Self::Unknown => "unknown-cmd",
            Self::BadArgument => "bad-arg",
        }
    }
}

/// A firmware-to-host event line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Event<'a> {
    /// Boot banner, sent after every reset. `rstfr` is the raw
    /// `RSTCTRL.RSTFR` value and `clk` names the boot clock.
    Boot {
        /// Raw reset-cause flags.
        rstfr: u8,
        /// Boot clock name, for example `rc4m`.
        clk: &'a str,
    },
    /// The command loop is up.
    Ready,
    /// Answer to a `PING`, echoing its number.
    Pong(u32),
    /// Acknowledges a `RUN`, echoing the test id the firmware parsed.
    RunAck {
        /// The parsed test id.
        id: &'a str,
    },
    /// One known test, in answer to `LIST`.
    Test {
        /// The test's wire name.
        id: &'a str,
    },
    /// Free-form diagnostics. Displayed only, never interpreted, except for
    /// the `<id> send` payload prompt of the echo tests.
    Log {
        /// The raw log body.
        body: &'a str,
    },
    /// The payload of an echo test, sent back for the host to compare.
    Echo {
        /// The payload as received by the firmware.
        payload: &'a str,
    },
    /// The single verdict line of one test run.
    Result {
        /// The test's wire name.
        id: &'a str,
        /// The verdict.
        outcome: Outcome,
        /// Optional single-token detail.
        detail: Option<&'a str>,
    },
    /// The firmware rejected an input line.
    Err {
        /// A short reason token.
        reason: &'a str,
    },
}

impl<'a> Event<'a> {
    /// Parses one firmware line. Returns [`None`] for any line without the
    /// sentinel prefix or with an unknown shape.
    #[must_use]
    pub fn parse(line: &'a str) -> Option<Self> {
        let rest = line.strip_prefix(SENTINEL)?.trim_end();
        let (word, tail) = split_word(rest);
        match word {
            "v1" => parse_boot(tail),
            "ready" if tail.is_empty() => Some(Self::Ready),
            "pong" => Some(Self::Pong(single_word(tail)?.parse().ok()?)),
            "run" => Some(Self::RunAck {
                id: single_word(tail)?,
            }),
            "test" => Some(Self::Test {
                id: single_word(tail)?,
            }),
            "log" if !tail.is_empty() => Some(Self::Log { body: tail }),
            "echo" => Some(Self::Echo { payload: tail }),
            "result" => parse_result(tail),
            "err" if !tail.is_empty() => Some(Self::Err { reason: tail }),
            _ => None,
        }
    }
}

fn parse_boot(tail: &str) -> Option<Event<'_>> {
    let rest = tail.strip_prefix("boot ")?;
    let (rstfr, rest) = split_word(rest);
    let rstfr = u8::from_str_radix(rstfr.strip_prefix("rstfr=0x")?, 16).ok()?;
    let clk = single_word(rest)?.strip_prefix("clk=")?;
    if clk.is_empty() {
        return None;
    }
    Some(Event::Boot { rstfr, clk })
}

fn parse_result(tail: &str) -> Option<Event<'_>> {
    let (id, rest) = split_word(tail);
    if id.is_empty() {
        return None;
    }
    let (outcome, rest) = split_word(rest);
    let outcome = Outcome::from_name(outcome)?;
    let detail = if rest.is_empty() { None } else { Some(rest) };
    Some(Event::Result {
        id,
        outcome,
        detail,
    })
}

/// Splits off the first space-separated word. The remainder keeps its inner
/// spacing.
fn split_word(s: &str) -> (&str, &str) {
    s.split_once(' ').unwrap_or((s, ""))
}

/// The whole input as one non-empty word, or [`None`].
fn single_word(s: &str) -> Option<&str> {
    if s.is_empty() || s.contains(' ') {
        return None;
    }
    Some(s)
}

#[cfg(feature = "ufmt")]
pub const fn hex_char(nibble: u8) -> char {
    let n = nibble & 0xF;
    let byte = if n < 10 { b'0' + n } else { b'A' + (n - 10) };
    byte as char
}

#[cfg(feature = "ufmt")]
impl ufmt::uDisplay for Event<'_> {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized,
    {
        ufmt::uwrite!(f, "{}", SENTINEL)?;
        match *self {
            Self::Boot { rstfr, clk } => ufmt::uwrite!(
                f,
                "v1 boot rstfr=0x{}{} clk={}",
                hex_char(rstfr >> 4),
                hex_char(rstfr),
                clk
            ),
            Self::Ready => ufmt::uwrite!(f, "ready"),
            Self::Pong(n) => ufmt::uwrite!(f, "pong {}", n),
            Self::RunAck { id } => ufmt::uwrite!(f, "run {}", id),
            Self::Test { id } => ufmt::uwrite!(f, "test {}", id),
            Self::Log { body } => ufmt::uwrite!(f, "log {}", body),
            Self::Echo { payload } => ufmt::uwrite!(f, "echo {}", payload),
            Self::Result {
                id,
                outcome,
                detail,
            } => {
                ufmt::uwrite!(f, "result {} {}", id, outcome.as_str())?;
                if let Some(detail) = detail {
                    ufmt::uwrite!(f, " {}", detail)?;
                }
                Ok(())
            }
            Self::Err { reason } => ufmt::uwrite!(f, "err {}", reason),
        }
    }
}
