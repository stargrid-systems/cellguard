//! Runtime logic for the `CellGuard` cellagent balancer MCU.
//!
//! The cellagent controls active balancing gates and reports temperature to the
//! cellcore over a UART link using the `cellguard-protocol`.
//!
//! [`CellagentRuntime`] drives the protocol: it decodes incoming COBS frames,
//! dispatches requests to the hardware traits [`GateControl`] and
//! [`TempSensor`], and writes encoded responses back to the bus.
//!
//! # Messages
//!
//! The cellagent answers two requests:
//!
//! - [`Kind::ReadTemperature`]: reads the temperature sensor and replies with a
//!   [`Kind::Temperature`] response carrying the value in centi-degrees Celsius
//!   as a little-endian `i16`.
//! - [`Kind::SetBalancer`]: drives the balancer gates from the 1-byte payload
//!   mask and replies with [`Kind::Ack`].
//!
//! Any other request receives a [`Kind::Nack`].
//!
//! [`Kind::ReadTemperature`]: cellguard_protocol::Kind::ReadTemperature
//! [`Kind::Temperature`]: cellguard_protocol::Kind::Temperature
//! [`Kind::SetBalancer`]: cellguard_protocol::Kind::SetBalancer
//! [`Kind::Ack`]: cellguard_protocol::Kind::Ack
//! [`Kind::Nack`]: cellguard_protocol::Kind::Nack

#![no_std]
#![warn(missing_docs)]

#[cfg(test)]
extern crate std;

pub use self::hw::{GateControl, TempSensor};
pub use self::runtime::CellagentRuntime;

mod hw;
mod runtime;
