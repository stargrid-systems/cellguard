//! `cellboot` is the hardware-independent core of the `CellGuard` bootloader and
//! firmware-update system.
//!
//! It defines the signed image format ([`image`]), the integrity and
//! authenticity primitives ([`crc32`], [`sha256`], [`hmac`]), and the I/O
//! traits ([`io`]) through which a concrete target performs all of its input
//! and output.
//!
//! The crate performs no I/O itself and depends on nothing outside of `core`.
//! The same logic therefore runs unchanged on the `AVR128` update agent, the
//! `ATtiny406` programmer, the `StorePilot`, and the host-side signing tool.
//!
//! # Authenticity model
//!
//! Images are authenticated with HMAC-SHA256 over the image header and payload.
//! The shared key protects against corrupt or untrusted firmware arriving over
//! the field bus. It is not meant to defend against an attacker who can
//! physically extract a device.
#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

pub use self::crc32::Crc32;
pub use self::hmac::Hmac;
pub use self::image::{ImageHeader, ImageKind, ParseError, Region, SignError, VerifyError, Verifier};
pub use self::mac::{Mac, ct_eq};
pub use self::protocol::{Command, NackReason, ProtocolError, Response};
pub use self::session::{RegionSlot, StagingLayout, UpdateAgent};
pub use self::sha256::Sha256;
pub use self::state::{AppHealth, PersistentState, StagedState, StateError, UpdateOutcome};

pub mod crc32;
pub mod hmac;
pub mod image;
pub mod io;
pub mod mac;
pub mod protocol;
pub mod session;
pub mod sha256;
pub mod state;
