//! On-target test implementations.
//!
//! Every test takes the shared [`Context`](crate::context::Context) and
//! returns its outcome plus an optional single-token detail. The dispatcher
//! in [`registry`](crate::registry) emits the result line.

pub mod clock;
pub mod spi_eeprom;
pub mod twi;
pub mod uart;
