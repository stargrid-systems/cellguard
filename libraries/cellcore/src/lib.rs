//! `cellcore` is the hardware-independent business logic for the `CellGuard`
//! core MCU (an AVR128).
//!
//! It is the home for everything the core MCU does. The logic is written
//! against abstract I/O traits (from `cellboot`), so nothing here touches a
//! register: a concrete firmware target supplies the peripherals and wires them
//! to these subsystems.
//!
//! # Subsystems
//!
//! - [`kat`]: the power-on crypto known-answer self-test, run at boot before
//!   any image or key is trusted.
//! - [`update`]: the field firmware-update agent. Receives a signed image over
//!   the bus, verifies it, and stages it for the `cellprog` programmer.
//!
//! Further subsystems live alongside [`update`] as the core-MCU logic grows.
#![no_std]
#![warn(missing_docs)]

pub mod kat;
pub mod update;
