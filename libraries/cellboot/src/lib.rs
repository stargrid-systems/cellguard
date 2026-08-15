//! `cellboot` is the hardware-independent core shared by the `CellGuard`
//! firmware-update roles.
//!
//! It defines the signed image format ([`image`]), the I/O traits ([`io`]),
//! the shared storage geometry ([`layout`]), the persistent updater state
//! ([`state`]), and the streaming programmer engine ([`programmer`]) through
//! which a concrete target performs all of its input and output. All roles
//! build on it: the `cellcore` update agent (runs on the core MCU), the
//! `cellprog` programmer (runs on the PROG MCU), and the bootloader itself.
//!
//! This crate holds no crypto. Signing, streaming verification, and the update
//! session machinery live in `cellcore`, so the `cellprog` programmer and the
//! bootloader link none of it.
//!
//! # Features
//!
//! Features are additive and off by default.
//!
//! - `drivers`: the shared `drivers::Cat25Store` external-EEPROM adapter.
//! - `avr128`: the AVR128 on-chip `drivers::EepromState` and
//!   `drivers::UserRowKeyStore` adapters. Pulls in `avrxt-hal`, so it only
//!   builds for the `avr-none` target.
//! - `testutil`: shared in-RAM test mocks (`testutil::MemStore`,
//!   `testutil::NullStateStore`, `testutil::SharedStore`). Test-only.
#![no_std]
#![warn(missing_docs)]

#[cfg(any(feature = "drivers", feature = "avr128"))]
pub mod drivers;
pub mod image;
pub mod io;
pub mod layout;
pub mod programmer;
pub mod state;
#[cfg(feature = "testutil")]
pub mod testutil;
