//! Hardware-independent core shared by the `CellGuard` firmware-update roles.
//!
//! Defines the signed image format ([`image`]), the factory identity record
//! ([`factory`]), the I/O traits ([`io`]), the shared storage geometry
//! ([`layout`]), the persistent updater state ([`state`]), and the streaming
//! programmer engine ([`programmer`]). All roles build on it: `cellcore`,
//! `cellprog`, and the bootloader. No crypto lives here, so `cellprog` and the
//! bootloader link none of it.
//!
//! # Features
//!
//! - `drivers`: the shared `drivers::Cat25Store` EEPROM adapter.
//! - `avr128`: the AVR128 on-chip `drivers` adapters. Pulls in `avrxt-hal`, so
//!   it only builds for `avr-none` targets.
//! - `testutil`: in-RAM test mocks. Test-only.
#![no_std]
#![warn(missing_docs)]

#[cfg(any(feature = "drivers", feature = "avr128"))]
pub mod drivers;
pub mod factory;
pub mod image;
pub mod io;
pub mod layout;
pub mod programmer;
pub mod state;
#[cfg(feature = "testutil")]
pub mod testutil;
