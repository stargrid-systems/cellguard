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
pub use self::prog::{ProgSource, ProgStatus};
#[cfg(feature = "bootloader")]
pub use self::session::{
    MAX_COMMAND_WIRE, MAX_REPLY_WIRE, PAGE_MAX, SessionStatus, SessionTarget, decode_begin,
    decode_page_data, decode_page_status, decode_read, decode_write, encode_begin,
    encode_page_data, encode_page_status, encode_read, encode_write,
};

mod cobs;
mod kind;
mod packet;
#[cfg(feature = "bootloader")]
mod prog;
#[cfg(feature = "bootloader")]
mod session;
