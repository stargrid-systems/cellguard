//! The field firmware-update agent.
//!
//! This subsystem answers bootloader commands from the field bus, streams a
//! received image into staging storage, and verifies it before marking it ready
//! to program. It never programs flash itself: after a successful commit,
//! [`session::UpdateAgent::pending_program`] tells the caller which region is
//! ready, so the caller can hand off to the `cellprog` programmer.
//!
//! The logic is written against the `cellboot` I/O traits only, so nothing here
//! touches a register. A concrete target supplies the storage, key, and
//! transport implementations.
//!
//! # Layout
//!
//! - [`session`]: the update-agent state machine ([`session::UpdateAgent`]).
//! - [`dispatch`]: the bus transport loop ([`dispatch::Dispatcher`]).
//! - [`command`]: the semantic command and response layer.
//! - [`state`]: the probe-able persistent state.
//! - [`verify`]: streaming image verification and host-side signing.
//! - [`mac`]: the message-authentication abstraction over HMAC-SHA256.
//!
//! # Authenticity model
//!
//! Images are authenticated with HMAC-SHA256 over the image header and payload.
//! The shared key protects against corrupt or untrusted firmware arriving over
//! the field bus. It is not meant to defend against an attacker who can
//! physically extract a device.

pub mod command;
pub mod dispatch;
pub mod mac;
pub mod session;
pub mod state;
pub mod verify;
