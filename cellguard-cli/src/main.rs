//! `cellguard-cli` is the host-side tool for the `CellGuard` field bus.
//!
//! It speaks the same COBS-framed protocol as the cellcore firmware, so a host
//! connected to the RS485 field bus (or directly to the cellcore's USART pins)
//! can push signed firmware images, probe device state, and read panic records.
//!
//! The most common use is pushing a cellagent image to demonstrate the full
//! update chain: the cellcore stages it, the cellprog reflashes U403 over UPDI.

use std::error::Error;
use std::path::PathBuf;
use std::process::ExitCode;
use std::{fs, io};

use cellboot::image::{ImageHeader, ImageKind, Region};
use cellboot::state::{PersistentState, STATE_LEN};
use cellcore::update::verify;
use cellguard_panic::{PanicRecord, RECORD_LEN};
use cellguard_protocol::Kind;
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
    /// Push a signed firmware image to the cellcore, which stages it and
    /// triggers the cellprog to flash the target.
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
        /// Fleet HMAC key as 32 hex chars (16 bytes). Defaults to all-0xFF
        /// (blank USERROW).
        #[arg(long)]
        key: Option<String>,
        /// Image `target_id`. Defaults to 1 for app/bootloader, 2 for
        /// cellagent.
        #[arg(long)]
        target_id: Option<u16>,
        /// Firmware version (informational). Defaults to 1.
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

    // BootBegin
    eprintln!("sending BootBegin...");
    let reply = transport.exchange(node, Kind::BootBegin, &signed_header)?;
    expect_ack(&reply, 0)?;

    // BootData chunks
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

    // BootCommit
    eprintln!("sending BootCommit...");
    let reply = transport.exchange(node, Kind::BootCommit, &[])?;
    expect_ack(&reply, payload.len() as u32)?;

    eprintln!("image staged successfully");
    match target {
        Target::App => eprintln!(
            "the cellcore bootloader will self-program on the next reset, or the cellprog will \
             reflash via UPDI on the next heartbeat-loss recovery"
        ),
        Target::Bootloader => eprintln!(
            "the cellcore will send ProgProgram to the cellprog, which reflashes the boot section \
             over UPDI"
        ),
        Target::Cellagent => eprintln!(
            "the cellcore has sent ProgProgram to the cellprog, which reflashes U403 over UPDI \
             via mux channel 3"
        ),
    }
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
