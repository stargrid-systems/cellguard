//! Host-side library for the `CellGuard` HIL test harness.
//!
//! It flashes the standalone test firmware (`hiltest-avr128da64`), runs the
//! on-target tests over the debug serial port, and restores the production
//! cellboot plus cellcore stack afterwards. The `hiltest` binary in this
//! package is a thin argument-parsing shell over [`commands`].

pub mod avrdude;
pub mod commands;
pub mod report;
pub mod serial;
pub mod session;
