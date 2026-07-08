//! The universal `CellGuard` bus protocol.
//!
//! This crate is shared by the bootloader, the application, and every MCU on
//! the RS485 daisy chain. It has two layers:
//!
//! - [`cobs`]: byte-at-a-time COBS framing for the transport.
//! - [`packet`]: a [`packet::Header`] (with its own CRC for cut-through routing)
//!   plus a payload and a separate payload CRC.
//!
//! Every message has a [`kind::Kind`] from one central registry, so a trace tool
//! can decode any packet. Bootloader kinds are gated behind the `bootloader`
//! feature but keep fixed discriminants.
#![no_std]
#![warn(missing_docs)]

pub mod cobs;
pub mod kind;
pub mod packet;
