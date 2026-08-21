//! `cellprog` is the hardware-independent programmer logic for the `CellGuard`
//! PROG MCU (the on-board `ATtiny406`).
//!
//! The PROG MCU never touches the field bus or the shared key. It executes
//! one transactional programming command at a time against a UPDI target
//! ([`session`]); the cellcore orchestrates what to program. The image was
//! already authenticated by the cellcore before staging, so there is no
//! crypto here.
//!
//! - [`session`] answers session commands over the local link: begin, page
//!   write, page read, end. It is the whole servant protocol in one
//!   host-testable state machine.
//! - [`supervisor`] answers whole-image program requests over the local link.
//!   Legacy, superseded by [`session`].
//! - [`writer`] is the UPDI-backed [`NvmWriter`](cellboot::io::NvmWriter)
//!   behind [`supervisor`].
#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

pub mod session;
pub mod supervisor;
pub mod writer;

pub use self::session::{Command, SessionHandler};
