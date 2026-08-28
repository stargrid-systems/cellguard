//! Host-side library for the `CellGuard` field bus tool.
//!
//! It speaks the cellcore's COBS-framed protocol over a serial link, so a
//! host can push signed firmware images, probe device state, and read panic
//! records. The `cellguard-cli` binary in this package is a thin
//! argument-parsing shell over [`commands`].
#![expect(
    clippy::float_arithmetic,
    reason = "host-only display scaling for telemetry values"
)]

pub mod commands;
pub mod push;
pub mod reply;
pub mod transport;
