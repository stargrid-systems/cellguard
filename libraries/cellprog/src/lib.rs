//! `cellprog` is the hardware-independent programmer logic for the `CellGuard`
//! PROG MCU (the on-board `ATtiny406`).
//!
//! The PROG MCU never touches the field bus or the shared key. It reads a
//! staged image from the external EEPROM
//! ([`ImageStore`](cellboot::io::ImageStore)), CRC-checks it, and writes it
//! into the main MCU over UPDI ([`NvmWriter`](cellboot::io::NvmWriter)). The
//! image was already authenticated by the main MCU before staging, so there is
//! no crypto here.
//!
//! - [`programmer`] streams a staged image into the target and verifies it.
//! - [`supervisor`] answers program requests over the local link. The firmware
//!   also uses it to recover the cellcore when its heartbeat is lost (reset,
//!   then reflash the staged application).
//! - [`writer`] is the UPDI-backed [`NvmWriter`](cellboot::io::NvmWriter) the
//!   programmer writes through, built on the `updi` crate.
//!
//! The image format and I/O traits come from the shared [`cellboot`] core.
#![no_std]
#![warn(missing_docs)]

pub mod programmer;
pub mod supervisor;
pub mod writer;
