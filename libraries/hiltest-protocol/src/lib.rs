//! Line protocol for the `CellGuard` HIL test harness.
//!
//! The HIL test firmware and the host runner exchange LF-terminated ASCII
//! lines over the debug UART. Every machine-readable firmware line starts
//! with the [`SENTINEL`] prefix, so the host parser survives glitch bytes
//! from the isolated console and free-form diagnostic text. Host commands
//! are bare words like `PING 7` or `RUN uart-echo-rc`.
//!
//! [`TestId`] is the shared registry of test names, [`Outcome`] the verdict
//! of one run, [`Command`] a parsed host line, and [`Event`] a parsed
//! firmware line. [`AckList`] is the structured detail of the `twi-scan`
//! result.
//!
//! ```
//! use hiltest_protocol::{Event, Outcome};
//!
//! let event = Event::parse("|HIL result uart-echo-rc PASS len=16").unwrap();
//! assert_eq!(
//!     event,
//!     Event::Result {
//!         id: "uart-echo-rc",
//!         outcome: Outcome::Pass,
//!         detail: Some("len=16"),
//!     }
//! );
//! ```
//!
//! # Features
//!
//! - `ufmt`: implement `ufmt::uDisplay` for [`Event`] and [`AckList`], so the
//!   firmware can emit protocol lines with `uwrite!`.
#![no_std]
#![warn(missing_docs)]

pub use self::detail::{AckList, SCAN_FIRST, SCAN_LAST};
pub use self::line::{Command, CommandError, Event, SENTINEL};
pub use self::test_id::{Outcome, TestId};

mod detail;
mod line;
mod test_id;
