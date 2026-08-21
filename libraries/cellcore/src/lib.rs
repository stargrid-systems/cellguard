//! `cellcore` is the hardware-independent business logic for the `CellGuard`
//! core MCU (an AVR128), written against the abstract `cellboot` I/O traits.
//! A firmware target supplies the peripherals.
//!
//! # Features
//!
//! Features are additive and off by default.
//!
//! - `sign`: host-side image signing (`update::verify::sign`). A device never
//!   signs, it only verifies.
//! - `telemetry`: the balancing-test layer (`balancing`).
#![no_std]
#![warn(missing_docs)]

pub mod kat;
pub mod update;

#[cfg(feature = "telemetry")]
pub mod balancing;
