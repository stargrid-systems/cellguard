//! `cellboot` is the hardware-independent core shared by the `CellGuard`
//! firmware-update roles.
//!
//! It defines the signed image format ([`image`]) and the I/O traits ([`io`])
//! through which a concrete target performs all of its input and output. Both
//! roles build on it: the `cellcore` update agent (runs on the core MCU) and the
//! `cellprog` programmer (runs on the PROG MCU).
//!
//! This crate holds no crypto. Signing, streaming verification, and the update
//! session machinery live in `cellcore`, so the `cellprog` programmer links none
//! of it.
//!
//! # Features
//!
//! Features are additive and off by default.
//!
//! - `drivers`: the shared `drivers::Cat25Store` external-EEPROM adapter.
//! - `avr128`: the AVR128 on-chip `drivers::EepromState` and
//!   `drivers::UserRowKeyStore` adapters. Pulls in `avrxt-hal`, so it only
//!   builds for the `avr-none` target.
#![no_std]
#![warn(missing_docs)]

#[cfg(any(feature = "drivers", feature = "avr128"))]
pub mod drivers;
pub mod image;
pub mod io;
