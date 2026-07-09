//! `cellboot` is the hardware-independent core of the `CellGuard` bootloader and
//! firmware-update system.
//!
//! It defines the signed image format ([`image`]), the persistent probe-able
//! state ([`state`]), the update-agent state machine ([`session`]), and the
//! I/O traits ([`io`]) through which a concrete target performs all of its
//! input and output.
//!
//! Integrity comes from the `crc` crate (CRC-16/CRC-32) and authenticity from
//! the `hmac-sha256` crate, wrapped by the local [`mac`] module which adds the
//! [`Mac`](mac::Mac) abstraction and constant-time comparison.
//!
//! The crate performs no I/O itself. The same logic runs unchanged on the
//! `AVR128` update agent, the `StorePilot`, and the host-side signing tool.
//!
//! # Authenticity model
//!
//! Images are authenticated with HMAC-SHA256 over the image header and payload.
//! The shared key protects against corrupt or untrusted firmware arriving over
//! the field bus. It is not meant to defend against an attacker who can
//! physically extract a device.
#![no_std]
#![warn(missing_docs)]

pub mod command;
pub mod dispatch;
pub mod image;
pub mod io;
pub mod mac;
pub mod programmer;
pub mod session;
pub mod state;
pub mod supervisor;
