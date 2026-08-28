//! `hiltest` drives the `CellGuard` HIL test harness from the host.
//!
//! It flashes the standalone test firmware (`hiltest-avr128da64`), runs the
//! on-target tests over the debug serial port, and restores the production
//! cellboot + cellcore stack afterwards. AVR firmware is normally built in
//! the project devcontainer: every flashing subcommand accepts prebuilt ELF
//! paths for the host-only workflow.

use std::error::Error;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::time::{Duration, Instant};

use clap::{Parser, Subcommand};
use hiltest_protocol::TestId;

use self::report::Summary;
use self::serial::Lines;
use self::session::Session;

mod avrdude;
mod report;
mod serial;
mod session;

const DEFAULT_BAUD: u32 = 115_200;
const PORT_ENV: &str = "HILTEST_PORT";

#[derive(Parser)]
#[command(
    name = "hiltest",
    version,
    about = "Host runner for the CellGuard HIL test harness"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Build the test firmware and flash it (chip erase included).
    Flash {
        /// Prebuilt hiltest ELF. Skips the local cargo build.
        #[arg(long)]
        elf: Option<PathBuf>,
    },
    /// Run tests against a board that runs the test firmware.
    Run {
        /// Serial port path. Falls back to the `HILTEST_PORT` environment
        /// variable.
        #[arg(long)]
        port: Option<String>,
        /// Serial baud rate.
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
        /// Build and flash the test firmware first.
        #[arg(long)]
        flash: bool,
        /// Prebuilt hiltest ELF for --flash. Skips the local cargo build.
        #[arg(long, requires = "flash")]
        elf: Option<PathBuf>,
        /// Per-test timeout in seconds. Keep it above the 8 s on-target
        /// deadman so a hang is reported by the reboot banner, not by this
        /// timeout.
        #[arg(long, default_value_t = 20)]
        timeout: u64,
        /// Test ids to run. Runs every known test when empty.
        tests: Vec<String>,
    },
    /// List the known test ids.
    List,
    /// Print raw lines from the serial port.
    Console {
        /// Serial port path. Falls back to the `HILTEST_PORT` environment
        /// variable.
        #[arg(long)]
        port: Option<String>,
        /// Serial baud rate.
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
    /// Restore the production stack: chip erase, BOOTSIZE fuse, cellboot,
    /// cellcore.
    Restore {
        /// Prebuilt cellboot ELF. Skips the local cargo build.
        #[arg(long)]
        boot_elf: Option<PathBuf>,
        /// Prebuilt cellcore ELF. Skips the local cargo build.
        #[arg(long)]
        core_elf: Option<PathBuf>,
    },
}

fn main() -> ExitCode {
    match run(Cli::parse()) {
        Ok(code) => code,
        Err(e) => {
            eprintln!("error: {e}");
            let mut source = e.source();
            while let Some(s) = source {
                eprintln!("  caused by: {s}");
                source = s.source();
            }
            ExitCode::FAILURE
        }
    }
}

fn run(cli: Cli) -> Result<ExitCode, Box<dyn Error>> {
    match cli.command {
        Command::Flash { elf } => {
            avrdude::flash_hiltest(elf.as_deref())?;
            Ok(ExitCode::SUCCESS)
        }
        Command::Run {
            port,
            baud,
            flash,
            elf,
            timeout,
            tests,
        } => run_tests(
            &resolve_port(port)?,
            baud,
            flash,
            elf.as_deref(),
            Duration::from_secs(timeout),
            &tests,
        ),
        Command::List => {
            for id in TestId::ALL {
                println!("{}", id.name());
            }
            Ok(ExitCode::SUCCESS)
        }
        Command::Console { port, baud } => console(&resolve_port(port)?, baud),
        Command::Restore { boot_elf, core_elf } => {
            avrdude::restore(boot_elf.as_deref(), core_elf.as_deref())?;
            Ok(ExitCode::SUCCESS)
        }
    }
}

fn resolve_port(arg: Option<String>) -> Result<String, Box<dyn Error>> {
    if let Some(port) = arg {
        return Ok(port);
    }
    std::env::var(PORT_ENV).map_err(|_| format!("no --port given and {PORT_ENV} is not set").into())
}

fn select_tests(names: &[String]) -> Result<Vec<TestId>, Box<dyn Error>> {
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

fn run_tests(
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
fn console(port: &str, baud: u32) -> Result<ExitCode, Box<dyn Error>> {
    let mut lines = Lines::open(port, baud)?;
    loop {
        let deadline = Instant::now() + Duration::from_secs(3600);
        if let Some(line) = lines.next_line(deadline)? {
            println!("{line}");
        }
    }
}
