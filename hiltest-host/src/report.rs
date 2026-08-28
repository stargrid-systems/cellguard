//! Per-test verdicts and the run summary.

use std::process::ExitCode;

use hiltest_protocol::TestId;

/// Host-side verdict of one test run.
pub enum Verdict {
    /// The firmware reported PASS and every host-side check held.
    Pass,
    /// The firmware reported FAIL, or a host-side check failed.
    Fail(Option<String>),
    /// The firmware skipped the test.
    Skip(Option<String>),
    /// No result line arrived within the per-test timeout.
    Timeout,
}

impl Verdict {
    const fn is_failure(&self) -> bool {
        matches!(self, Self::Fail(_) | Self::Timeout)
    }

    fn describe(&self) -> String {
        match self {
            Self::Pass => "PASS".to_owned(),
            Self::Fail(detail) => with_detail("FAIL", detail.as_deref()),
            Self::Skip(detail) => with_detail("SKIP", detail.as_deref()),
            Self::Timeout => "TIMEOUT".to_owned(),
        }
    }
}

fn with_detail(word: &str, detail: Option<&str>) -> String {
    detail.map_or_else(|| word.to_owned(), |d| format!("{word} ({d})"))
}

/// Collected verdicts of one `hiltest run`.
pub struct Summary {
    entries: Vec<(TestId, Verdict)>,
}

impl Default for Summary {
    fn default() -> Self {
        Self::new()
    }
}

impl Summary {
    /// Creates an empty summary.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            entries: Vec::new(),
        }
    }

    pub fn record(&mut self, id: TestId, verdict: Verdict) {
        self.entries.push((id, verdict));
    }

    /// Prints one line per test plus a count line.
    pub fn print(&self) {
        println!();
        for (id, verdict) in &self.entries {
            println!("{:<24} {}", id.name(), verdict.describe());
        }
        let failed = self.entries.iter().filter(|(_, v)| v.is_failure()).count();
        let skipped = self
            .entries
            .iter()
            .filter(|(_, v)| matches!(v, Verdict::Skip(_)))
            .count();
        let passed = self.entries.len() - failed - skipped;
        println!("{passed} passed, {failed} failed, {skipped} skipped");
    }

    /// Nonzero exactly when a test failed or timed out.
    #[must_use]
    pub fn exit_code(&self) -> ExitCode {
        if self.entries.iter().any(|(_, v)| v.is_failure()) {
            ExitCode::FAILURE
        } else {
            ExitCode::SUCCESS
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// [`ExitCode`] has no `PartialEq`, so compare the debug forms.
    fn assert_code(actual: ExitCode, expected: ExitCode) {
        assert_eq!(format!("{actual:?}"), format!("{expected:?}"));
    }

    #[test]
    fn describe_covers_every_verdict() {
        assert_eq!(Verdict::Pass.describe(), "PASS");
        assert_eq!(Verdict::Fail(None).describe(), "FAIL");
        assert_eq!(
            Verdict::Fail(Some("echo-mismatch".to_owned())).describe(),
            "FAIL (echo-mismatch)"
        );
        assert_eq!(
            Verdict::Skip(Some("unprovisioned".to_owned())).describe(),
            "SKIP (unprovisioned)"
        );
        assert_eq!(Verdict::Timeout.describe(), "TIMEOUT");
    }

    #[test]
    fn with_detail_appends_the_detail_in_parentheses() {
        assert_eq!(with_detail("FAIL", None), "FAIL");
        assert_eq!(with_detail("SKIP", Some("later")), "SKIP (later)");
    }

    #[test]
    fn exit_code_succeeds_when_every_test_passes_or_skips() {
        let mut summary = Summary::new();
        summary.record(TestId::UartEchoRc, Verdict::Pass);
        summary.record(TestId::IdentRead, Verdict::Skip(Some("x".to_owned())));
        assert_code(summary.exit_code(), ExitCode::SUCCESS);
    }

    #[test]
    fn exit_code_fails_on_a_failed_verdict() {
        let mut summary = Summary::new();
        summary.record(TestId::UartEchoRc, Verdict::Pass);
        summary.record(TestId::TwiScan, Verdict::Fail(None));
        assert_code(summary.exit_code(), ExitCode::FAILURE);
    }

    #[test]
    fn exit_code_fails_on_a_timeout() {
        let mut summary = Summary::new();
        summary.record(TestId::ClockExtclk, Verdict::Timeout);
        assert_code(summary.exit_code(), ExitCode::FAILURE);
    }

    #[test]
    fn exit_code_succeeds_on_an_empty_run() {
        assert_code(Summary::new().exit_code(), ExitCode::SUCCESS);
    }
}
