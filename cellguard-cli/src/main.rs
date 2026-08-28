//! `cellguard-cli` is the host-side tool for the `CellGuard` field bus.
//!
//! This binary is a thin argument-parsing shell. The logic lives in the
//! `cellguard_cli` library crate.

use std::error::Error;
use std::path::PathBuf;
use std::process::ExitCode;

use cellguard_cli::commands::{self, InaRef, SnapshotKind, Target};
use cellguard_protocol::Kind;
use clap::{Parser, Subcommand};

const DEFAULT_BAUD: u32 = 115_200;
const DEFAULT_CHUNK: usize = 128;
const DEFAULT_FW_VERSION: u32 = 1;

#[derive(Parser)]
#[command(
    name = "cellguard-cli",
    version,
    about = "Host tool for the CellGuard field bus"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand)]
enum Command {
    /// Push a signed firmware image to the cellcore.
    PushImage {
        /// Serial port path (e.g. /dev/ttyUSB0).
        #[arg(long)]
        port: String,
        /// Target cellcore node address.
        #[arg(long)]
        node: u8,
        /// Path to the raw payload (.bin) file.
        payload: PathBuf,
        /// Which region to target.
        #[arg(long)]
        target: Target,
        /// Fleet HMAC key as 32 hex chars (16 bytes), default all-0xFF
        /// (blank USERROW).
        #[arg(long)]
        key: Option<String>,
        /// Image `target_id` (default 1, or 2 for cellagent).
        #[arg(long)]
        target_id: Option<u16>,
        /// Firmware version (informational).
        #[arg(long, default_value_t = DEFAULT_FW_VERSION)]
        fw_version: u32,
        /// Serial baud rate.
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
        /// `BootData` chunk size in bytes.
        #[arg(long, default_value_t = DEFAULT_CHUNK)]
        chunk_size: usize,
    },
    /// Probe the cellcore for its current updater state.
    Probe {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
    /// Query a node's identity: board model and revision, serial number,
    /// firmware version.
    Identity {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
    /// Read the last panic record from a node.
    PanicProbe {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
    /// Read the cell-voltage snapshot (raw codes plus millivolts).
    Cells {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
    /// Read the balance-current snapshot (raw codes plus amperes).
    Currents {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
        /// S501 `INA_REF` position: `gnd` (unipolar) or `3v3` (bipolar).
        #[arg(long, value_enum, default_value_t = InaRef::Gnd)]
        ina_ref: InaRef,
    },
    /// Read the supply rails.
    Rails {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
    /// Read the temperature sensors.
    Temps {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
    /// Read the full balancing status frame.
    Balance {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
    /// Set the bleed-leg enable masks (`en_3r6`, `en_36r5`).
    SetBleed {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
        /// Leg-A (2.0 R) enable mask, bit x = cell x+1.
        #[arg(long)]
        en_3r6: u8,
        /// Leg-B (7.2 R) enable mask, bit x = cell x+1.
        #[arg(long)]
        en_36r5: u8,
    },
    /// Set the bleed PWM duty in 1/65536 units (TCD0 WOD on PB7, ~1.5 kHz).
    SetBleedPwm {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
        #[arg(long)]
        duty: u16,
    },
    /// Set power-enable flags (bits: 1=`ACTIVE_BALANCER_ON`, 2=`EN_ALL`,
    /// 4=`POWER_ON`).
    SetPower {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
        #[arg(long)]
        flags: u8,
    },
    /// Assert (1) or release (0) the hardware gate-off.
    GateOff {
        #[arg(long)]
        port: String,
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
        #[arg(long)]
        on: u8,
    },
    /// Command the cellagent balancer gates (routed through the cellcore).
    SetBalancer {
        #[arg(long)]
        port: String,
        /// Cellagent node address (the cellcore routes it downstream).
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
        /// Gate mask: bit 0 `GATE_A`, bit 1 `GATE_B`, bit 2 `ALL_OFF`.
        #[arg(long)]
        mask: u8,
    },
    /// Read the cellagent's last commanded gate mask (routed).
    GateState {
        #[arg(long)]
        port: String,
        /// Cellagent node address (the cellcore routes it downstream).
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
}

fn main() -> ExitCode {
    let cli = Cli::parse();
    match run(cli) {
        Ok(()) => ExitCode::SUCCESS,
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

fn run(cli: Cli) -> Result<(), Box<dyn Error>> {
    match cli.command {
        Command::PushImage {
            port,
            node,
            payload,
            target,
            key,
            target_id,
            fw_version,
            baud,
            chunk_size,
        } => commands::push_image(
            &port, node, &payload, target, key, target_id, fw_version, baud, chunk_size,
        ),
        Command::Probe { port, node, baud } => commands::probe(&port, node, baud),
        Command::Identity { port, node, baud } => commands::identity(&port, node, baud),
        Command::PanicProbe { port, node, baud } => commands::panic_probe(&port, node, baud),
        Command::Cells { port, node, baud } => {
            commands::read_snapshot(&port, node, baud, SnapshotKind::Cells, InaRef::Gnd)
        }
        Command::Currents {
            port,
            node,
            baud,
            ina_ref,
        } => commands::read_snapshot(&port, node, baud, SnapshotKind::Currents, ina_ref),
        Command::Rails { port, node, baud } => commands::rails(&port, node, baud),
        Command::Temps { port, node, baud } => commands::temps(&port, node, baud),
        Command::Balance { port, node, baud } => commands::balance_status(&port, node, baud),
        Command::SetBleed {
            port,
            node,
            baud,
            en_3r6,
            en_36r5,
        } => commands::ack(&port, node, baud, Kind::SetBleed, &[en_3r6, en_36r5]),
        Command::SetBleedPwm {
            port,
            node,
            baud,
            duty,
        } => commands::ack(&port, node, baud, Kind::SetBleedPwm, &duty.to_le_bytes()),
        Command::SetPower {
            port,
            node,
            baud,
            flags,
        } => commands::ack(&port, node, baud, Kind::SetPower, &[flags]),
        Command::GateOff {
            port,
            node,
            baud,
            on,
        } => commands::ack(&port, node, baud, Kind::GateOff, &[on]),
        Command::SetBalancer {
            port,
            node,
            baud,
            mask,
        } => commands::ack(&port, node, baud, Kind::SetBalancer, &[mask]),
        Command::GateState { port, node, baud } => commands::gate_state(&port, node, baud),
    }
}
