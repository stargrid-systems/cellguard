//! Firmware builds and avrdude invocations.
//!
//! Every step checks the child's exit status. No output is scraped: a grep
//! in a pipe eats the exit status and flashes stale binaries.
//!
//! AVR firmware is normally built inside the project devcontainer, while
//! avrdude and the serial port live on the host. The local `cargo build` is
//! therefore best effort: when it cannot run (no avr-gcc on the host), the
//! caller is pointed at the `--elf` style overrides.

use std::error::Error;
use std::path::{Path, PathBuf};
use std::process::Command;

const PROGRAMMER: &str = "pickit_basic_updi";
const PART: &str = "avr128da64";

/// Builds the test firmware (unless `elf` is given) and flashes it with a
/// chip erase.
pub fn flash_hiltest(elf: Option<&Path>) -> Result<(), Box<dyn Error>> {
    let elf = match elf {
        Some(path) => existing(path)?,
        None => build_or_explain("hiltest-avr128da64", "--elf")?,
    };
    run(
        avrdude().args(["-e", "-U"]).arg(flash_op(&elf)),
        "avrdude (flash test image)",
    )
}

/// Restores the production stack: chip erase, BOOTSIZE fuse, cellboot,
/// cellcore.
pub fn restore(boot_elf: Option<&Path>, core_elf: Option<&Path>) -> Result<(), Box<dyn Error>> {
    let boot = resolve("cellboot-avr128da64", boot_elf, "--boot-elf")?;
    let core = resolve("cellcore-avr128da64", core_elf, "--core-elf")?;
    // The chip erase resets fuses and USERROW, so BOOTSIZE is rewritten in
    // the same invocation.
    run(
        avrdude().args(["-e", "-U", "bootsize:w:0x10:m"]),
        "avrdude (chip erase + BOOTSIZE fuse)",
    )?;
    run(
        avrdude().args(["-D", "-U"]).arg(flash_op(&boot)),
        "avrdude (flash cellboot)",
    )?;
    run(
        avrdude().args(["-D", "-U"]).arg(flash_op(&core)),
        "avrdude (flash cellcore)",
    )
}

/// Picks the ELF for one production workspace: the override, a fresh local
/// build, or a previously built ELF at the conventional path.
fn resolve(name: &str, elf: Option<&Path>, flag: &str) -> Result<PathBuf, Box<dyn Error>> {
    if let Some(path) = elf {
        return existing(path);
    }
    let build_err = match build(name) {
        Ok(elf) => return Ok(elf),
        Err(e) => e,
    };
    let conventional = elf_path(name);
    if conventional.is_file() {
        eprintln!(
            "note: local build failed, using the existing ELF at {}",
            conventional.display()
        );
        return Ok(conventional);
    }
    Err(format!(
        "{build_err}\nno ELF at {} either. Build {name} in the devcontainer or pass {flag} <path>",
        conventional.display()
    )
    .into())
}

/// Builds `name` locally, or explains the `--elf` escape hatch.
fn build_or_explain(name: &str, flag: &str) -> Result<PathBuf, Box<dyn Error>> {
    build(name).map_err(|e| {
        format!(
            "{e}\nthe AVR toolchain usually lives in the devcontainer. Build {name} there and \
             pass {flag} {}",
            elf_path(name).display()
        )
        .into()
    })
}

/// Runs `cargo build --release` in the workspace and returns the ELF path.
fn build(name: &str) -> Result<PathBuf, Box<dyn Error>> {
    run(
        Command::new("cargo")
            .args(["build", "--release"])
            .current_dir(workspace_dir(name)),
        "cargo build",
    )?;
    existing(&elf_path(name))
}

fn existing(path: &Path) -> Result<PathBuf, Box<dyn Error>> {
    if path.is_file() {
        Ok(path.to_path_buf())
    } else {
        Err(format!("ELF not found at {}", path.display()).into())
    }
}

/// The repository root, resolved from this crate's location at compile time.
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..")
}

fn workspace_dir(name: &str) -> PathBuf {
    repo_root().join(name)
}

/// The conventional release ELF location of an AVR workspace.
fn elf_path(name: &str) -> PathBuf {
    workspace_dir(name)
        .join("target/avr-none/release")
        .join(name)
}

fn avrdude() -> Command {
    let mut command = Command::new("avrdude");
    command.args(["-c", PROGRAMMER, "-p", PART]);
    command
}

fn flash_op(elf: &Path) -> String {
    format!("flash:w:{}:e", elf.display())
}

/// Runs a child with inherited stdio and checks its exit status.
fn run(command: &mut Command, what: &str) -> Result<(), Box<dyn Error>> {
    let status = command
        .status()
        .map_err(|e| format!("{what}: failed to start: {e}"))?;
    if status.success() {
        Ok(())
    } else {
        Err(format!("{what} failed with {status}").into())
    }
}
