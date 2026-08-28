//! The CLI command implementations.

use std::error::Error;
use std::path::Path;
use std::process::ExitCode;
use std::time::{Duration, Instant};

use hiltest_protocol::TestId;

use crate::avrdude;
use crate::report::Summary;
use crate::serial::Lines;
use crate::session::Session;

/// Resolves test names to ids. An empty selection means every known test.
///
/// # Errors
///
/// Returns an error when a name is not a known test id.
pub fn select_tests(names: &[String]) -> Result<Vec<TestId>, Box<dyn Error>> {
    if names.is_empty() {
        return Ok(TestId::ALL.to_vec());
    }
    names
        .iter()
        .map(|name| {
            TestId::from_name(name).ok_or_else(|| format!("unknown test id: {name}").into())
        })
        .collect()
}

/// Runs the selected tests over `port`, flashing the test firmware first
/// when `flash` is set.
///
/// # Errors
///
/// Returns an error when a test name is unknown, the flash fails, or the
/// serial session breaks.
pub fn run_tests(
    port: &str,
    baud: u32,
    flash: bool,
    elf: Option<&Path>,
    timeout: Duration,
    tests: &[String],
) -> Result<ExitCode, Box<dyn Error>> {
    let selected = select_tests(tests)?;
    if flash {
        avrdude::flash_hiltest(elf)?;
    }
    let mut session = Session::open(port, baud)?;
    session.wait_ready(flash)?;
    let mut summary = Summary::new();
    for id in selected {
        println!("running {}", id.name());
        let verdict = session.run_test(id, timeout)?;
        summary.record(id, verdict);
    }
    summary.print();
    Ok(summary.exit_code())
}

/// Dumb line viewer. Runs until interrupted.
///
/// # Errors
///
/// Returns an error when the port cannot be opened or a read fails.
pub fn console(port: &str, baud: u32) -> Result<ExitCode, Box<dyn Error>> {
    let mut lines = Lines::open(port, baud)?;
    loop {
        let deadline = Instant::now() + Duration::from_secs(3600);
        if let Some(line) = lines.next_line(deadline)? {
            println!("{line}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn select_tests_defaults_to_all() {
        let selected = select_tests(&[]).unwrap();
        assert_eq!(selected, TestId::ALL.to_vec());
    }

    #[test]
    fn select_tests_resolves_known_names() {
        let names = ["clock-extclk".to_owned(), "uart-echo-24m".to_owned()];
        let selected = select_tests(&names).unwrap();
        assert_eq!(selected, vec![TestId::ClockExtclk, TestId::UartEcho24m]);
    }

    #[test]
    fn select_tests_rejects_an_unknown_name() {
        let names = ["clock-extclk".to_owned(), "no-such-test".to_owned()];
        let err = select_tests(&names).unwrap_err();
        assert_eq!(err.to_string(), "unknown test id: no-such-test");
    }
}
