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

impl Summary {
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
    pub fn exit_code(&self) -> ExitCode {
        if self.entries.iter().any(|(_, v)| v.is_failure()) {
            ExitCode::FAILURE
        } else {
            ExitCode::SUCCESS
        }
    }
}
