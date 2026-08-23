//! `cellcore` is the hardware-independent business logic for the `CellGuard`
//! core MCU (an AVR128), written against the abstract `cellboot` I/O traits.
//! A firmware target supplies the peripherals.
//!
//! # Features
//!
//! - `sign`: host-side image signing (`update::verify::sign`). A device never
//!   signs, it only verifies, so host tools and tests enable this.
//! - `telemetry`: the balancing-test layer (`balancing`). Pulls in the protocol
//!   crate's telemetry kinds.
#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

pub mod kat;
pub mod update;

#[cfg(feature = "telemetry")]
pub mod balancing;
