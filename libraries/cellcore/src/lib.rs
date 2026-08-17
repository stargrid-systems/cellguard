//! `cellcore` is the hardware-independent business logic for the `CellGuard`
//! core MCU (an AVR128).
//!
//! The logic is written against abstract I/O traits (from `cellboot`), so
//! nothing here touches a register: a concrete firmware target supplies the
//! peripherals and wires them to these subsystems.
//!
//! # Subsystems
//!
//! - [`kat`]: the power-on crypto known-answer self-test, run at boot before
//!   any image or key is trusted.
//! - [`update`]: the field firmware-update agent. Receives a signed image over
//!   the bus, verifies it, and stages it for the `cellprog` programmer.
//! - [`balancing`]: the balancing-test telemetry and actuator layer (behind the
//!   `telemetry` feature).
//!
//! # Features
//!
//! Features are additive and off by default.
//!
//! - `sign`: host-side image signing (`update::verify::sign`). A device never
//!   signs, it only verifies. Enabled by host tools and tests.
//! - `telemetry`: the balancing-test layer (`balancing`), which pulls in the
//!   telemetry kinds of `cellguard-protocol`.
#![no_std]
#![warn(missing_docs)]

pub mod kat;
pub mod update;

#[cfg(feature = "telemetry")]
pub mod balancing;
