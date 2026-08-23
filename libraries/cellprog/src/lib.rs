//! Hardware-independent programmer logic for the `CellGuard` PROG MCU.
//!
//! The PROG MCU executes one programming command at a time against a UPDI
//! target and never touches the field bus or the shared key. Images are
//! authenticated by the cellcore before staging, so there is no crypto here.
//!
//! - [`session`]: the servant-side session protocol.
//! - [`supervisor`]: whole-image program requests. Legacy, superseded by
//!   [`session`].
//! - [`writer`]: the UPDI-backed [`NvmWriter`](cellboot::io::NvmWriter) behind
//!   [`supervisor`].
//!
//! # Features
//!
//! - `page-read`: flash read-back over the session protocol. It does not fit
//!   the `ATtiny406` servant next to the rest of its firmware, so the feature
//!   is off there.
#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

pub mod session;
pub mod supervisor;
pub mod writer;

pub use self::session::{Command, SessionHandler};
