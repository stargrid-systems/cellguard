//! Small, table-free CRC implementations for corruption detection.
//!
//! Two variants are provided, each in its own module and re-exported here:
//!
//! - [`Crc32`]: reflected CRC-32 (IEEE 802.3), for firmware image integrity.
//! - [`Crc16`]: CRC-16/MODBUS, for the communication frame.
//!
//! Both are bitwise and table-free to keep code size small on the target, and
//! both stream: feeding data in chunks yields the same result as feeding it all
//! at once. These detect corruption only, not tampering.
#![no_std]
#![warn(missing_docs)]

pub use self::crc16::{Crc16, checksum16};
pub use self::crc32::{Crc32, checksum32};

mod bitwise;
mod crc16;
mod crc32;
