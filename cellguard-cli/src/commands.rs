//! The CLI command implementations.
//!
//! Every function here opens a serial port, exchanges packets with a node,
//! and prints the result. The pure helpers live in [`crate::push`] and
//! [`crate::reply`].

use std::error::Error;
use std::path::Path;
use std::{fs, io};

use cellboot::image::{ImageHeader, ImageKind, Region};
use cellboot::state::PersistentState;
use cellcore::update::verify;
use cellguard_panic::{PanicRecord, RECORD_LEN};
use cellguard_protocol::{
    BOARD_MODEL_UNPROVISIONED, BalancerStatus, DeviceId, Kind, RAIL_ORDER, RAILS, RailSnapshot,
    SerialNumber, Snapshot, TEMP_ORDER, TempSnapshot,
};
use hmac_sha256::HMAC;

use crate::push::{data_frames, parse_key};
use crate::reply::{expect_ack, nack_reason, parse_state};
use crate::transport::Transport;

const DEFAULT_TARGET_ID: u16 = 1;
const DEFAULT_CELLAGENT_TARGET_ID: u16 = 2;
const DEFAULT_KEY_HEX: &str = "ffffffffffffffffffffffffffffffff";

/// The image region a push targets.
#[derive(Clone, Copy, clap::ValueEnum)]
pub enum Target {
    /// The cellcore application region.
    App,
    /// The cellcore bootloader region.
    Bootloader,
    /// The cellagent application.
    Cellagent,
}

impl Target {
    const fn region(self) -> Region {
        match self {
            Self::App => Region::ApplicationCode,
            Self::Bootloader => Region::Bootloader,
            Self::Cellagent => Region::CellagentApp,
        }
    }

    const fn kind(self) -> ImageKind {
        match self {
            Self::App | Self::Cellagent => ImageKind::Application,
            Self::Bootloader => ImageKind::Bootloader,
        }
    }

    const fn default_target_id(self) -> u16 {
        match self {
            Self::Cellagent => DEFAULT_CELLAGENT_TARGET_ID,
            Self::App | Self::Bootloader => DEFAULT_TARGET_ID,
        }
    }
}

/// Signs a payload and streams it to a node in `BootData` chunks.
///
/// # Errors
///
/// Returns an error if the key or payload is invalid, the port fails, or the
/// device rejects any step of the session.
///
/// # Panics
///
/// Panics if `chunk_size` is zero.
#[expect(
    clippy::too_many_arguments,
    reason = "mirrors the push-image CLI options"
)]
#[expect(clippy::cast_possible_truncation, reason = "checked at function entry")]
pub fn push_image(
    port: &str,
    node: u8,
    payload_path: &Path,
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
    for frame in data_frames(&payload, chunk_size) {
        let reply = transport.exchange(node, Kind::BootData, &frame.bytes)?;
        expect_ack(&reply, frame.end_offset)?;
        eprint!("\ruploading: {}/{total} bytes", frame.end_offset);
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

/// Probes a node for its current updater state and prints it.
///
/// # Errors
///
/// Returns an error if the port fails or the reply is not a valid
/// `BootStatus`.
pub fn probe(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
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

/// Queries a node's identity and prints board model, serial, and firmware.
///
/// # Errors
///
/// Returns an error if the port fails or a reply has the wrong kind or shape.
pub fn identity(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let id_reply = transport.exchange(node, Kind::ReadDeviceId, &[])?;
    if id_reply.kind != Kind::DeviceId {
        return Err(format!("expected DeviceId, got {:?}", id_reply.kind).into());
    }
    let id = DeviceId::decode(&id_reply.payload).ok_or("device-id payload has the wrong shape")?;

    let serial_reply = transport.exchange(node, Kind::ReadSerialNumber, &[])?;
    if serial_reply.kind != Kind::SerialNumber {
        return Err(format!("expected SerialNumber, got {:?}", serial_reply.kind).into());
    }
    let serial =
        SerialNumber::decode(&serial_reply.payload).ok_or("serial payload has the wrong shape")?;

    if id.board_model == BOARD_MODEL_UNPROVISIONED {
        println!("board  : unprovisioned (no factory record)");
    } else {
        println!(
            "board  : model {} rev {}",
            id.board_model, id.board_revision
        );
    }
    println!("fw     : {}", id.fw_version);
    print!("serial : ");
    for byte in serial.serial {
        print!("{byte:02X}");
    }
    println!();
    Ok(())
}

/// Reads and prints the last panic record from a node.
///
/// # Errors
///
/// Returns an error if the port fails or the reply is not a `PanicStatus`.
pub fn panic_probe(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, Kind::PanicProbe, &[])?;
    match reply.kind {
        Kind::PanicStatus => {
            if reply.payload.is_empty() {
                println!("no panic record");
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

fn print_state(state: &PersistentState) {
    println!("{:<16}: {}", "agent_version", state.agent_version);
    println!("{:<16}: {}", "app_version", state.app_version);
    println!("{:<16}: {:?}", "app_health", state.app_health);
    println!("{:<16}: {:?}", "staged", state.staged);
    println!("{:<16}: {:?}", "staged_region", state.staged_region);
    println!("{:<16}: {}", "staged_version", state.staged_version);
    println!("{:<16}: {:?}", "last_outcome", state.last_outcome);
    println!("{:<16}: {}", "boot_count", state.boot_count);
    println!("{:<16}: {}", "program_attempts", state.program_attempts);
}

fn print_panic(record: &PanicRecord) {
    let file = core::str::from_utf8(
        record
            .file
            .get(..usize::from(record.file_len))
            .unwrap_or(&[]),
    )
    .unwrap_or("<invalid utf8>");
    println!("panic at {file}:{}:{}", record.line, record.col);
    println!("  reset_flags     : 0x{:02X}", record.reset_flags);
    println!("  consecutive     : {}", record.consecutive_panics);
}

/// Which telemetry snapshot to read.
#[derive(Clone, Copy)]
pub enum SnapshotKind {
    /// Cell voltages.
    Cells,
    /// Balance currents.
    Currents,
}

/// ADS131M08 internal reference in millivolts and full scale.
const ADS_VREF_MV: f32 = 1200.0;
const ADS_FULL_SCALE: f32 = 8_388_608.0;
/// Cell-voltage divider ratio (820k:28k).
const CELL_DIVIDER: f32 = 0.0330;
/// INA190 transfer: volts out per ampere (47 mOhm shunt, gain 25).
const INA_V_PER_A: f32 = 1.175;
/// INA190 REF midpoint in bipolar mode (S501 at 3.15 V).
const INA_REF_MIDPOINT_V: f32 = 3.15;

/// The S501 `INA_REF` bench position the current readings assume.
#[derive(Clone, Copy, clap::ValueEnum)]
pub enum InaRef {
    /// Unipolar: zero current reads zero.
    #[value(name = "gnd")]
    Gnd,
    /// Bipolar: the output centers at the INA190 REF midpoint.
    #[value(name = "3v3")]
    V3v3,
}

/// Converts an INA190 output voltage into amperes through the balance shunt.
///
/// # Examples
///
/// ```
/// use cellguard_cli::commands::{InaRef, ina_amps};
///
/// let amps = ina_amps(1.175, InaRef::Gnd);
/// assert!((amps - 1.0).abs() < 1e-6);
/// ```
#[must_use]
pub fn ina_amps(volts: f32, ina_ref: InaRef) -> f32 {
    let offset = match ina_ref {
        InaRef::Gnd => 0.0,
        InaRef::V3v3 => INA_REF_MIDPOINT_V,
    };
    (volts - offset) / INA_V_PER_A
}

/// Reads and prints a cell-voltage or balance-current snapshot.
///
/// # Errors
///
/// Returns an error if the port fails or the reply has the wrong kind or
/// shape.
pub fn read_snapshot(
    port: &str,
    node: u8,
    baud: u32,
    what: SnapshotKind,
    ina_ref: InaRef,
) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let kind = match what {
        SnapshotKind::Cells => Kind::ReadCellVoltages,
        SnapshotKind::Currents => Kind::ReadBalanceCurrents,
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
                let amps = ina_amps(volts, ina_ref);
                println!("cell {}: raw {code}, {amps:.3} A", i + 1);
            }
        }
    }
    Ok(())
}

/// Rail ADC reference in millivolts (10-bit MCU ADC).
const RAIL_VREF_MV: f32 = 1800.0;
const RAIL_FULL_SCALE: f32 = 1023.0;
/// Rail divider ratios in `RAIL_ORDER`.
const RAIL_DIVIDER: [f32; RAILS] = [0.052, 0.052, 0.0536, 0.0536, 0.0536, 0.5, 0.0536, 0.0536];

/// Reads and prints the supply rails.
///
/// # Errors
///
/// Returns an error if the port fails or the reply has the wrong kind or
/// shape.
pub fn rails(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, Kind::ReadRails, &[])?;
    if reply.kind != Kind::Rails {
        return Err(format!("expected Rails, got {:?}", reply.kind).into());
    }
    let snap = RailSnapshot::decode(&reply.payload).ok_or("rails payload has the wrong shape")?;
    for ((name, code), ratio) in RAIL_ORDER.iter().zip(snap.codes).zip(RAIL_DIVIDER) {
        let mv = f32::from(code) / RAIL_FULL_SCALE * RAIL_VREF_MV / ratio;
        println!("{name}: {code} ({mv:.0} mV)");
    }
    Ok(())
}

/// Reads and prints the temperature sensors.
///
/// # Errors
///
/// Returns an error if the port fails or the reply has the wrong kind or
/// shape.
pub fn temps(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
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

/// Reads and prints the full balancing status frame.
///
/// # Errors
///
/// Returns an error if the port fails or the reply has the wrong kind or
/// shape.
pub fn balance_status(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
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

/// Sends a command and expects a bare `Ack`.
///
/// # Errors
///
/// Returns an error if the port fails, the node nacks, or the reply has any
/// other kind.
pub fn ack(
    port: &str,
    node: u8,
    baud: u32,
    kind: Kind,
    payload: &[u8],
) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, kind, payload)?;
    match reply.kind {
        Kind::Ack => Ok(()),
        Kind::Nack => Err(format!("nacked: {}", nack_reason(&reply.payload)).into()),
        other => Err(format!("expected Ack, got {other:?}").into()),
    }
}

/// Reads and prints the cellagent's last commanded gate mask.
///
/// # Errors
///
/// Returns an error if the port fails or the reply has the wrong kind.
pub fn gate_state(port: &str, node: u8, baud: u32) -> Result<(), Box<dyn Error>> {
    let mut transport = Transport::open(port, baud)?;
    let reply = transport.exchange(node, Kind::ReadBalancerGateState, &[])?;
    if reply.kind != Kind::BalancerGateState {
        return Err(format!("expected BalancerGateState, got {:?}", reply.kind).into());
    }
    let mask = reply.payload.first().copied().unwrap_or(0);
    println!("gate mask: {mask:#04x}");
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{InaRef, ina_amps};

    #[test]
    fn ina_amps_gnd_scales_by_transfer() {
        assert!((ina_amps(1.175, InaRef::Gnd) - 1.0).abs() < 1e-6);
        assert!((ina_amps(0.5875, InaRef::Gnd) - 0.5).abs() < 1e-6);
    }

    #[test]
    fn ina_amps_gnd_zero_reads_zero() {
        assert!(ina_amps(0.0, InaRef::Gnd).abs() < 1e-6);
    }

    #[test]
    fn ina_amps_gnd_keeps_sign() {
        assert!((ina_amps(-1.175, InaRef::Gnd) + 1.0).abs() < 1e-6);
    }

    #[test]
    fn ina_amps_bipolar_centers_at_midpoint() {
        assert!(ina_amps(3.15, InaRef::V3v3).abs() < 1e-6);
        assert!((ina_amps(4.325, InaRef::V3v3) - 1.0).abs() < 1e-5);
        assert!(ina_amps(2.0, InaRef::V3v3) < 0.0);
    }
}
