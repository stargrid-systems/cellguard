//! A hardware-independent UPDI host for programming modern-AVR (`AVRxt`)
//! targets.
//!
//! UPDI is Microchip's single-wire debug and programming interface. This crate
//! speaks the host side of it: it drives a target's UPDI slave to unlock NVM,
//! erase, and write program memory. It touches no registers of its own. All
//! wire I/O goes through the [`UpdiLink`] transport trait, which a concrete
//! target implements (typically a USART in one-wire mode). That keeps the whole
//! stack testable off-target.
//!
//! # Layers
//!
//! - [`UpdiLink`]: the one transport seam. Send bytes (echo consumed) and
//!   receive bytes, plus a BREAK.
//! - [`Updi`]: the link layer. The UPDI instruction set (`LDCS`, `STCS`, `LDS`,
//!   `STS`, `LD`, `ST`, `REPEAT`, `KEY`) over a [`UpdiLink`].
//! - [`Programmer`]: the programming layer for AVR Dx (NVMCTRL v2). Unlock,
//!   reset into programming mode, erase, write, and read back flash.
//!
//! # Features
//!
//! - `mock`: exposes `mock::MockTarget`, a UPDI target emulator, so downstream
//!   crates can integration-test their own [`UpdiLink`] consumers. Test-only.

#![no_std]
#![warn(missing_docs)]

pub use self::link::{Updi, UpdiError, UpdiLink};
pub use self::programmer::{PAGE_SIZE, ProgError, Programmer};

mod link;
#[cfg(any(test, feature = "mock"))]
pub mod mock;
mod programmer;
