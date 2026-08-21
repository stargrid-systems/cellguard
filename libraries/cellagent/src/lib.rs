//! Runtime logic for the `CellGuard` cellagent balancer MCU.
//!
//! The cellagent controls active balancing gates and reports temperature to the
//! cellcore over a UART link using the `cellguard-protocol`.
//!
//! [`CellagentRuntime`] drives the protocol: it decodes incoming COBS frames,
//! dispatches requests to the hardware traits [`GateControl`] and
//! [`TempSensor`], and writes encoded responses back to the bus. Unknown or
//! malformed requests receive a `Nack`.

#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

pub use self::hw::{GateControl, TempSensor};
pub use self::runtime::{CellagentRuntime, DEFAULT_GATE_TIMEOUT_TICKS, SAFE_GATE_MASK};

mod hw;
mod runtime;
