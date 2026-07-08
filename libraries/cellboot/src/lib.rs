//! `cellboot` is the hardware-independent core of the `CellGuard` bootloader and
//! firmware-update system.
//!
//! It defines the signed image format ([`image`]), the persistent probe-able
//! state ([`state`]), the update-agent state machine ([`session`]), and the
//! I/O traits ([`io`]) through which a concrete target performs all of its
//! input and output.
//!
//! Integrity and authenticity primitives live in sibling crates: `crc` for
//! CRC-16/CRC-32 and `sha256` for SHA-256, HMAC-SHA256, and the MAC
//! abstraction.
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

pub use self::image::{ImageHeader, ImageKind, ParseError, Region, SignError, VerifyError, Verifier};
pub use self::protocol::{Command, NackReason, ProtocolError, Response};
pub use self::session::{RegionSlot, StagingLayout, UpdateAgent};
pub use self::state::{AppHealth, PersistentState, StagedState, StateError, UpdateOutcome};

pub mod image;
pub mod io;
pub mod protocol;
pub mod session;
pub mod state;
