//! Runtime logic for the `CellGuard` cellagent balancer MCU.
//!
//! [`CellagentRuntime`] decodes COBS frames from the cellcore over the UART
//! link, drives the gates and temperature sensor through [`GateControl`]
//! and [`TempSensor`], and writes responses back to the bus.

#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

pub use self::hw::{GateControl, TempSensor};
pub use self::runtime::{CellagentRuntime, DEFAULT_GATE_TIMEOUT_TICKS, SAFE_GATE_MASK};

mod hw;
mod runtime;
