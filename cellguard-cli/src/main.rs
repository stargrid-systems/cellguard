//! `cellguard-cli` is the host-side tool for the `CellGuard` field bus.
//!
//! It speaks the cellcore's COBS-framed protocol over a serial link, so a
//! host can push signed firmware images, probe device state, and read panic
//! records.

use std::error::Error;
use std::path::PathBuf;
use std::process::ExitCode;
use std::{fs, io};

use cellboot::image::{ImageHeader, ImageKind, Region};
use cellboot::state::{PersistentState, STATE_LEN};
use cellcore::update::verify;
use cellguard_panic::{PanicRecord, RECORD_LEN};
use cellguard_protocol::{
    BalancerStatus, Kind, RAIL_ORDER, RailSnapshot, Snapshot, TEMP_ORDER, TempSnapshot,
};
use clap::{Parser, Subcommand};
use hmac_sha256::HMAC;

use self::transport::{Reply, Transport};

mod transport;

const DEFAULT_BAUD: u32 = 115_200;
const DEFAULT_CHUNK: usize = 128;
const DEFAULT_TARGET_ID: u16 = 1;
const DEFAULT_CELLAGENT_TARGET_ID: u16 = 2;
const DEFAULT_FW_VERSION: u32 = 1;
const DEFAULT_KEY_HEX: &str = "ffffffffffffffffffffffffffffffff";

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
        #[arg(long)]
        node: u8,
        #[arg(long, default_value_t = DEFAULT_BAUD)]
        baud: u32,
    },
}

#[derive(Clone, Copy, clap::ValueEnum)]
enum Target {
    /// The cellcore application region.
    App,
    /// The cellcore bootloader region.
    Bootloader,
    /// The cellagent application.
    Cellagent,
}

impl Target {
    fn region(self) -> Region {
        match self {
            Self::App => Region::ApplicationCode,
            Self::Bootloader => Region::Bootloader,
            Self::Cellagent => Region::CellagentApp,
        }
    }

    fn kind(self) -> ImageKind {
        match self {
            Self::App | Self::Cellagent => ImageKind::Application,
            Self::Bootloader => ImageKind::Bootloader,
        }
    }

    fn default_target_id(self) -> u16 {
        match self {
            Self::Cellagent => DEFAULT_CELLAGENT_TARGET_ID,
            Self::App | Self::Bootloader => DEFAULT_TARGET_ID,
        }
    }
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
        } => push_image(
            &port, node, &payload, target, key, target_id, fw_version, baud, chunk_size,
        ),
        Command::Probe { port, node, baud } => probe(&port, node, baud),
        Command::PanicProbe { port, node, baud } => panic_probe(&port, node, baud),
        Command::Cells { port, node, baud } => {
            read_snapshot(&port, node, baud, SnapshotKind::Cells)
        }
        Command::Currents { port, node, baud } => {
            read_snapshot(&port, node, baud, SnapshotKind::Currents)
        }
        Command::Rails { port, node, baud } => rails(&port, node, baud),
        Command::Temps { port, node, baud } => temps(&port, node, baud),
        Command::Balance { port, node, baud } => balance_status(&port, node, baud),
        Command::SetBleed {
            port,
            node,
            baud,
            en_3r6,
            en_36r5,
        } => ack(&port, node, baud, Kind::SetBleed, &[en_3r6, en_36r5]),
        Command::SetBleedPwm {
            port,
            node,
            baud,
            duty,
        } => ack(&port, node, baud, Kind::SetBleedPwm, &duty.to_le_bytes()),
        Command::SetPower {
            port,
            node,
            baud,
            flags,
        } => ack(&port, node, baud, Kind::SetPower, &[flags]),
        Command::GateOff {
            port,
            node,
            baud,
            on,
        } => ack(&port, node, baud, Kind::GateOff, &[on]),
        Command::SetBalancer {
            port,
            node,
            baud,
            mask,
        } => ack(&port, node, baud, Kind::SetBalancer, &[mask]),
        Command::GateState { port, node, baud } => gate_state(&port, node, baud),
    }
}

#[allow(clippy::too_many_arguments)]
#[expect(clippy::cast_possible_truncation, reason = "checked at function entry")]
fn push_image(
    port: &str,
    node: u8,
    payload_path: &PathBuf,
    target: Target,
    key_hex: Option<String>,
    target_id: Option<u16>,
    fw_version: u32,
    baud: u32,
    chunk_size: usize,
) -> Result<(), Box<dyn Error>> {
    let key = parse_key(&key_hex.unwrap_or_else(|| DEFAULT_KEY_HEX.to_string()))?;
    let target_id = target_id.unwrap_or_else(|| target.default_target_id());
    let payload = fs::read(payload_path)?;
    if payload.len() > u32::MAX as usize {
        return Err(format!(
            "payload too large: {} bytes (max {})",
            payload.len(),
            u32::MAX
        )
        .into());
    }

    eprintln!(
        "payload: {} bytes, region {:?}, target_id {}, fw_version {}",
        payload.len(),
        target.region(),
        target_id,
        fw_version
    );

    let header = ImageHeader {
        kind: target.kind(),
        region: target.region(),
        target_id,
        fw_version,
        payload_len: 0,
        payload_crc32: 0,
        hmac: [0u8; 32],
    };
    let signed_header = verify::sign(header, HMAC::new(key), &payload)
        .map_err(|e| format!("signing failed: {e}"))?;

    let mut transport = Transport::open(port, baud)?;

    eprintln!("sending BootBegin...");
    let reply = transport.exchange(node, Kind::BootBegin, &signed_header)?;
    expect_ack(&reply, 0)?;

    let total = payload.len();
    let mut offset = 0usize;
    let mut data_buf = vec![0u8; 4 + chunk_size];
    for chunk in payload.chunks(chunk_size) {
        data_buf[..4].copy_from_slice(&(offset as u32).to_le_bytes());
        data_buf[4..4 + chunk.len()].copy_from_slice(chunk);
        let reply = transport.exchange(node, Kind::BootData, &data_buf[..4 + chunk.len()])?;
        let expected = offset + chunk.len();
        expect_ack(&reply, expected as u32)?;
        offset = expected;
        eprint!("\ruploading: {offset}/{total} bytes");
        let _ = io::Write::flush(&mut io::stderr());
    }
    eprintln!();

    eprintln!("sending BootCommit...");
    let reply = transport.exchange(node, Kind::BootCommit, &[])?;
    expect_ack(&reply, payload.len() as u32)?;

    eprintln!("image staged successfully");
    let epilogue = match target {
        Target::App => "the cellcore bootloader self-programs the image on the next reset",
        Target::Bootloader => {
            "the image stays staged in the boot EEPROM: flashing the boot section is a bench-only \
             step"
        }
        Target::Cellagent => {
            "the cellcore streams the image to the cellprog over the session link, which reflashes \
             U403 over UPDI via mux channel 3"
        }
    };
    eprintln!("{epilogue}");
    Ok(())
}

fn probe(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, Kind::BootProbe, &[])?;
    match reply.kind {
        Kind::BootStatus => {
            let state = parse_state(&reply.payload)?;
            print_state(&state);
            Ok(())
        }
        other => Err(format!("expected BootStatus, got {other:?}").into()),
    }
}

fn panic_probe(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, Kind::PanicProbe, &[])?;
    match reply.kind {
        Kind::PanicStatus => {
            if reply.payload.is_empty() {
                eprintln!("no panic record");
            } else if reply.payload.len() == RECORD_LEN {
                let bytes: &[u8; RECORD_LEN] = reply
                    .payload
                    .as_slice()
                    .try_into()
                    .map_err(|_| "panic record has wrong length")?;
                match PanicRecord::parse(bytes) {
                    Ok(record) => print_panic(&record),
                    Err(e) => eprintln!("panic record parse failed: {e}"),
                }
            } else {
                eprintln!(
                    "unexpected panic-status payload length: {}",
                    reply.payload.len()
                );
            }
            Ok(())
        }
        other => Err(format!("expected PanicStatus, got {other:?}").into()),
    }
}

fn expect_ack(reply: &Reply, expected_offset: u32) -> Result<(), Box<dyn Error>> {
    match reply.kind {
        Kind::BootAck => {
            let next_offset = reply
                .payload
                .get(..4)
                .and_then(|b| b.try_into().ok())
                .map_or(0, u32::from_le_bytes);
            if next_offset != expected_offset {
                return Err(format!(
                    "unexpected next_offset: expected {expected_offset}, got {next_offset}"
                )
                .into());
            }
            Ok(())
        }
        Kind::BootNack => {
            let reason = reply
                .payload
                .first()
                .and_then(|&c| cellcore::update::command::NackReason::from_code(c));
            Err(format!("device rejected: {reason:?}").into())
        }
        other => Err(format!("expected BootAck/BootNack, got {other:?}").into()),
    }
}

fn parse_state(payload: &[u8]) -> Result<PersistentState, Box<dyn Error>> {
    let bytes: &[u8; STATE_LEN] = payload
        .get(..STATE_LEN)
        .and_then(|s| s.try_into().ok())
        .ok_or("status payload is not STATE_LEN bytes")?;
    PersistentState::parse(bytes).map_err(|e| format!("state parse failed: {e}").into())
}

fn print_state(state: &PersistentState) {
    eprintln!("agent_version : {}", state.agent_version);
    eprintln!("app_version   : {}", state.app_version);
    eprintln!("app_health    : {:?}", state.app_health);
    eprintln!("staged        : {:?}", state.staged);
    eprintln!("staged_region : {:?}", state.staged_region);
    eprintln!("staged_version: {}", state.staged_version);
    eprintln!("last_outcome  : {:?}", state.last_outcome);
    eprintln!("boot_count    : {}", state.boot_count);
    eprintln!("program_attempts: {}", state.program_attempts);
}

fn print_panic(record: &PanicRecord) {
    let file = core::str::from_utf8(&record.file[..usize::from(record.file_len)])
        .unwrap_or("<invalid utf8>");
    eprintln!("panic at {file}:{}:{}", record.line, record.col);
    eprintln!("  reset_flags     : 0x{:02X}", record.reset_flags);
    eprintln!("  consecutive     : {}", record.consecutive_panics);
}

fn parse_key(hex: &str) -> Result<[u8; 16], Box<dyn Error>> {
    if hex.len() != 32 {
        return Err(format!("key must be 32 hex chars (16 bytes), got {}", hex.len()).into());
    }
    let mut key = [0u8; 16];
    for (i, byte) in key.iter_mut().enumerate() {
        let hi = hex_val(hex.as_bytes()[2 * i])?;
        let lo = hex_val(hex.as_bytes()[2 * i + 1])?;
        *byte = (hi << 4) | lo;
    }
    Ok(key)
}

fn hex_val(c: u8) -> Result<u8, Box<dyn Error>> {
    match c {
        b'0'..=b'9' => Ok(c - b'0'),
        b'a'..=b'f' => Ok(c - b'a' + 10),
        b'A'..=b'F' => Ok(c - b'A' + 10),
        _ => Err(format!("invalid hex char: {}", char::from(c)).into()),
    }
}

#[derive(Clone, Copy)]
enum SnapshotKind {
    Cells,
    Currents,
}

/// ADS131M08 internal reference in millivolts and full scale.
const ADS_VREF_MV: f32 = 1200.0;
const ADS_FULL_SCALE: f32 = 8_388_608.0;
/// Cell-voltage divider ratio (820k:28k).
const CELL_DIVIDER: f32 = 0.0330;
/// INA190 transfer: volts out per ampere (47 mOhm shunt, gain 25).
const INA_V_PER_A: f32 = 1.175;

fn read_snapshot(
    port: &str,
    node: u8,
    baud: u32,
    what: SnapshotKind,
) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let (kind, label) = match what {
        SnapshotKind::Cells => (Kind::ReadCellVoltages, "cell"),
        SnapshotKind::Currents => (Kind::ReadBalanceCurrents, "current"),
    };
    let reply = transport.exchange(node, kind, &[])?;
    let expected = match what {
        SnapshotKind::Cells => Kind::CellVoltages,
        SnapshotKind::Currents => Kind::BalanceCurrents,
    };
    if reply.kind != expected {
        return Err(format!("expected {expected:?}, got {:?}", reply.kind).into());
    }
    let snap = Snapshot::decode(&reply.payload).ok_or("snapshot payload has the wrong shape")?;
    println!("seq {}", snap.seq);
    for (i, code) in snap.codes.iter().enumerate() {
        #[expect(clippy::cast_precision_loss, reason = "24-bit codes fit f32 exactly")]
        let volts = *code as f32 / ADS_FULL_SCALE * ADS_VREF_MV / 1000.0;
        match what {
            SnapshotKind::Cells => {
                let cell_mv = volts / CELL_DIVIDER * 1000.0;
                println!("cell {}: raw {code}, {:.1} mV", i + 1, cell_mv);
            }
            SnapshotKind::Currents => {
                let amps = volts / INA_V_PER_A;
                println!("cell {}: raw {code}, {amps:.3} A", i + 1);
            }
        }
    }
    let _ = label;
    Ok(())
}

fn rails(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, Kind::ReadRails, &[])?;
    if reply.kind != Kind::Rails {
        return Err(format!("expected Rails, got {:?}", reply.kind).into());
    }
    let snap = RailSnapshot::decode(&reply.payload).ok_or("rails payload has the wrong shape")?;
    for (name, code) in RAIL_ORDER.iter().zip(snap.codes) {
        println!("{name}: {code}");
    }
    Ok(())
}

fn temps(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, Kind::ReadTemperatures, &[])?;
    if reply.kind != Kind::Temperatures {
        return Err(format!("expected Temperatures, got {:?}", reply.kind).into());
    }
    let temps = TempSnapshot::decode(&reply.payload).ok_or("temps payload has the wrong shape")?;
    for (name, centi) in TEMP_ORDER.iter().zip(temps.temps) {
        if centi == cellguard_protocol::TEMP_INVALID {
            println!("{name}: unavailable");
        } else {
            println!("{name}: {}.{:02} C", centi / 100, centi % 100);
        }
    }
    Ok(())
}

fn balance_status(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, Kind::ReadBalancerStatus, &[])?;
    if reply.kind != Kind::BalancerStatus {
        return Err(format!("expected BalancerStatus, got {:?}", reply.kind).into());
    }
    let status =
        BalancerStatus::decode(&reply.payload).ok_or("status payload has the wrong shape")?;
    println!("en_3r6 mask: {:#04x}", status.en_3r6);
    println!("en_36r5 mask: {:#04x}", status.en_36r5);
    println!(
        "pwm duty: {} ({:.2}%)",
        status.pwm_duty,
        f32::from(status.pwm_duty) / 655.36
    );
    println!("gate mask: {:#04x}", status.gate_mask);
    println!("tiny_all_off: {}", status.tiny_all_off);
    println!("emergency_gate_off: {}", status.emergency_gate_off);
    println!("active_balancer_on: {}", status.active_balancer_on);
    println!("en_all: {}", status.en_all);
    println!("cellagent_alive: {}", status.cellagent_alive);
    Ok(())
}

fn ack(port: &str, node: u8, baud: u32, kind: Kind, payload: &[u8]) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, kind, payload)?;
    match reply.kind {
        Kind::Ack => Ok(()),
        Kind::Nack => Err(format!("nacked: {:?}", reply.payload).into()),
        other => Err(format!("expected Ack, got {other:?}").into()),
    }
}

fn gate_state(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, Kind::ReadBalancerGateState, &[])?;
    if reply.kind != Kind::BalancerGateState {
        return Err(format!("expected BalancerGateState, got {:?}", reply.kind).into());
    }
    let mask = reply.payload.first().copied().unwrap_or(0);
    println!("gate mask: {mask:#04x}");
    Ok(())
}
