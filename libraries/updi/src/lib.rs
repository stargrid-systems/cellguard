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
//!   receive bytes, plus a BREAK. Lives in `link`.
//! - [`Updi`]: the driver. The UPDI instruction set (`LDCS`, `STCS`, `LDS`,
//!   `STS`, `LD`, `ST`, `REPEAT`, `KEY`) over a [`UpdiLink`]. Lives in
//!   `driver`.
//! - [`Programmer`]: the programming layer for AVR Dx (NVMCTRL v2). Unlock,
//!   reset into programming mode, erase, write, and read back flash.
//! - [`TinyProgrammer`]: the same for tinyAVR 0/1-series (NVMCTRL v0/v1).
//!
//! # Features
//!
//! - `mock`: exposes `mock::MockTarget`, a UPDI target emulator, so downstream
//!   crates can integration-test their own [`UpdiLink`] consumers. Test-only.

#![no_std]
#![warn(missing_docs)]

pub use self::driver::{Updi, UpdiError};
pub use self::link::UpdiLink;
pub use self::programmer::{PAGE_SIZE, ProgError, Programmer};
pub use self::tiny::TinyProgrammer;

mod driver;
mod link;
#[cfg(any(test, feature = "mock"))]
pub mod mock;
mod programmer;
pub mod tiny;
