//! The field firmware-update agent.
//!
//! It answers bootloader commands from the field bus, streams the received
//! image into staging storage, verifies it, and marks it ready. It never
//! programs flash itself: after a successful commit, the caller hands the
//! region off to the `cellprog` programmer. The logic is written against the
//! `cellboot` I/O traits only.
//!
//! # Layout
//!
//! - [`session`]: the update-agent state machine ([`session::UpdateAgent`]).
//! - [`dispatch`]: the bus transport loop ([`dispatch::Dispatcher`]).
//! - [`command`]: the semantic command and response layer.
//! - [`handoff`]: the request that tells the `cellprog` programmer to flash a
//!   committed image.
//! - [`verify`]: streaming image verification and host-side signing.
//! - [`mac`]: the message-authentication abstraction over HMAC-SHA256.
//!
//! Images are authenticated with HMAC-SHA256 under a shared key. This
//! protects against corrupt or untrusted firmware arriving over the field
//! bus, not against an attacker who can physically extract a device.

pub mod command;
pub mod dispatch;
pub mod handoff;
pub mod mac;
pub mod session;
pub mod session_driver;
pub mod verify;
