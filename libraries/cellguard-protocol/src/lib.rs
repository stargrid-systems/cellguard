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

pub use self::cobs::{DecodeError, Decoder, Encoder, encode_frame};
pub use self::kind::Kind;
pub use self::packet::{Error, HEADER_LEN, Header, PAYLOAD_CRC_LEN, Packet};

mod cobs;
mod kind;
mod packet;
