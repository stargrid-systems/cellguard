//! The universal `CellGuard` bus protocol.
//!
//! This crate is shared by the bootloader, the application, and every MCU on
//! the RS485 daisy chain. It has two layers:
//!
//! - COBS framing for the transport ([`Encoder`], [`Decoder`]).
//! - a [`Packet`] carrying a [`Header`] (with its own CRC for cut-through
//!   routing) plus a payload and a separate payload CRC.
//!
//! Every message has a [`Kind`] from one central registry, so a trace tool can
//! decode any packet. Bootloader kinds are gated behind the `bootloader`
//! feature but keep fixed discriminants.
#![no_std]
#![warn(missing_docs)]

pub use self::cobs::{DecodeError, Decoder, Encoder, encode_frame, max_encoded_len};
pub use self::kind::Kind;
pub use self::packet::{Error, HEADER_LEN, Header, PAYLOAD_CRC_LEN, Packet};
#[cfg(feature = "bootloader")]
pub use self::session::{
    Command, MAX_COMMAND_WIRE, MAX_REPLY_WIRE, PAGE_MAX, Reply, SessionCmd, SessionStatus,
    SessionTarget, decode_begin, decode_command, decode_page_data, decode_page_status,
    decode_reply, decode_write, encode_begin, encode_command, encode_page_data, encode_page_status,
    encode_reply, encode_write,
};
#[cfg(all(feature = "bootloader", feature = "page-read"))]
pub use self::session::{decode_read, encode_read};
#[cfg(feature = "telemetry")]
pub use self::telemetry::{
    BalancerStatus, BleedMasks, CELLS, POWER_ACTIVE_BALANCER, POWER_EN_ALL, RAIL_ORDER, RAILS,
    RailSnapshot, Seq, Snapshot, TEMP_INVALID, TEMP_ORDER, TEMPS, TempSnapshot, decode_bleed,
    decode_pwm, decode_temps, encode_temps,
};

mod cobs;
mod kind;
mod packet;
#[cfg(feature = "bootloader")]
mod session;
#[cfg(feature = "telemetry")]
mod telemetry;
