//! `hiltest` drives the `CellGuard` HIL test harness from the host.
//!
//! This binary is a thin argument-parsing shell. The logic lives in the
//! `hiltest_host` library crate. AVR firmware is normally built in the
//! project devcontainer: every flashing subcommand accepts prebuilt ELF
//! paths for the host-only workflow.

use std::error::Error;
use std::path::PathBuf;
use std::process::ExitCode;
use std::time::Duration;

use clap::{Parser, Subcommand};
use hiltest_host::{avrdude, commands};
use hiltest_protocol::TestId;

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
        } => commands::run_tests(
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
        Command::Console { port, baud } => commands::console(&resolve_port(port)?, baud),
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
