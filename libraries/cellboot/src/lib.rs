//! Hardware-independent core shared by the `CellGuard` firmware-update roles.
//!
//! Defines the signed image format ([`image`]), the I/O traits ([`io`]), the
//! shared storage geometry ([`layout`]), the persistent updater state
//! ([`state`]), and the streaming programmer engine ([`programmer`]). All
//! roles build on it: `cellcore`, `cellprog`, and the bootloader. No crypto
//! lives here, so `cellprog` and the bootloader link none of it.
//!
//! # Features
//!
//! Additive and off by default.
//!
//! - `drivers`: the shared `drivers::Cat25Store` EEPROM adapter.
//! - `avr128`: the AVR128 on-chip `drivers` adapters. Pulls in `avrxt-hal`, so
//!   `avr-none` targets only.
//! - `testutil`: in-RAM test mocks. Test-only.
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
