//! Hardware-independent programmer logic for the `CellGuard` PROG MCU.
//!
//! The PROG MCU executes one transactional programming command at a time
//! against a UPDI target and never touches the field bus or the shared
//! key. The cellcore orchestrates what to program and streams the image
//! over the local link, page by page. Images are authenticated by the
//! cellcore before staging, so there is no crypto here.
//!
//! - [`session`]: the servant-side session protocol.
#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

pub mod session;

pub use self::session::{Command, SessionHandler};
