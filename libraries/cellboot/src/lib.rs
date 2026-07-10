//! `cellboot` is the hardware-independent core of the `CellGuard` bootloader and
//! firmware-update system.
//!
//! It defines the signed image format ([`image`]) and the I/O traits ([`io`])
//! through which a concrete target performs all of its input and output. These
//! are shared by both firmware roles: the AVR128 update agent and the `cellprog`
//! programmer.
//!
//! # Features
//!
//! Features are additive and off by default.
//!
//! - `agent`: the update-agent side that runs on the AVR128. Adds the state
//!   machine ([`session`]), the transport loop ([`dispatch`]), the semantic
//!   command layer ([`command`]), the probe-able persistent state ([`state`]),
//!   and authentication ([`mac`], plus `image::Verifier` and
//!   `image::ImageHeader::sign`). Pulls in `hmac-sha256`, `crc`, and
//!   `cellguard-protocol`. The `cellprog` programmer never enables this, so it
//!   never links the crypto code.
//! - `drivers`: the shared `drivers::Cat25Store` adapter.
//! - `avr128`: the AVR128 on-chip `drivers::EepromState` and
//!   `drivers::UserRowKeyStore` adapters. Pulls in `avrxt-hal`, so it only
//!   builds for the `avr-none` target.
//!
//! # Authenticity model
//!
//! Images are authenticated with HMAC-SHA256 over the image header and payload.
//! The shared key protects against corrupt or untrusted firmware arriving over
//! the field bus. It is not meant to defend against an attacker who can
//! physically extract a device.
#![no_std]
#![warn(missing_docs)]

#[cfg(feature = "agent")]
pub mod command;
#[cfg(feature = "agent")]
pub mod dispatch;
#[cfg(any(feature = "drivers", feature = "avr128"))]
pub mod drivers;
pub mod image;
pub mod io;
#[cfg(feature = "agent")]
pub mod mac;
#[cfg(feature = "agent")]
pub mod session;
#[cfg(feature = "agent")]
pub mod state;
