//! The common programming interface implemented by every UPDI target family.
//!
//! [`Programmer`] (AVR Dx, NVMCTRL v2) and [`TinyProgrammer`] (tinyAVR 0/1,
//! NVMCTRL v0/v1) expose the same five operations with different command
//! values and page sizes. [`FlashProg`] factors that shared surface into a
//! single trait, so a downstream writer (the cellprog `NvmWriter`) can drive
//! either family through one generic type.
//!
//! The trait is sealed: new target families get their own impl inside this
//! crate.

use crate::link::UpdiLink;
use crate::programmer::{ProgError, Programmer};
use crate::tiny::TinyProgrammer;

mod private {
    pub trait Sealed {}
    impl<L: super::UpdiLink> Sealed for super::Programmer<L> {}
    impl<L: super::UpdiLink> Sealed for super::TinyProgrammer<L> {}
}

/// The common programming interface for a UPDI flash target.
///
/// Each call maps one-to-one onto the underlying programmer. The page size is
/// an associated constant so callers can size their erase bookkeeping at
/// compile time.
pub trait FlashProg: private::Sealed {
    /// Flash page size in bytes.
    const PAGE_SIZE: u32;
    /// Error type reported by the programmer.
    type Error;

    /// Enters programming mode (halts the target).
    ///
    /// # Errors
    ///
    /// See the implementor.
    fn enter(&mut self) -> Result<(), Self::Error>;
    /// Erases the flash page containing `page_base` (a page-aligned address).
    ///
    /// # Errors
    ///
    /// See the implementor.
    fn erase_page(&mut self, page_base: u32) -> Result<(), Self::Error>;
    /// Writes `data` to flash at `addr`. Pages touched must be erased first.
    ///
    /// # Errors
    ///
    /// See the implementor.
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), Self::Error>;
    /// Reads `buf.len()` bytes back from flash at `addr`.
    ///
    /// # Errors
    ///
    /// See the implementor.
    fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Self::Error>;
    /// Leaves programming mode and lets the target run.
    ///
    /// # Errors
    ///
    /// See the implementor.
    fn leave(&mut self) -> Result<(), Self::Error>;
}

impl<L: UpdiLink> FlashProg for Programmer<L> {
    const PAGE_SIZE: u32 = crate::programmer::PAGE_SIZE;
    type Error = ProgError<L::Error>;

    // Inherent methods shadow the trait methods of the same name, so these
    // call the underlying `Programmer` implementation rather than recursing.
    fn enter(&mut self) -> Result<(), Self::Error> {
        self.enter()
    }
    fn erase_page(&mut self, page_base: u32) -> Result<(), Self::Error> {
        self.erase_flash_page(page_base)
    }
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.write_flash(addr, data)
    }
    fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.read_flash(addr, buf)
    }
    fn leave(&mut self) -> Result<(), Self::Error> {
        self.leave()
    }
}

impl<L: UpdiLink> FlashProg for TinyProgrammer<L> {
    const PAGE_SIZE: u32 = crate::tiny::PAGE_SIZE;
    type Error = ProgError<L::Error>;

    fn enter(&mut self) -> Result<(), Self::Error> {
        self.enter()
    }
    fn erase_page(&mut self, page_base: u32) -> Result<(), Self::Error> {
        self.erase_flash_page(page_base)
    }
    fn write(&mut self, addr: u32, data: &[u8]) -> Result<(), Self::Error> {
        self.write_flash(addr, data)
    }
    fn read(&mut self, addr: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.read_flash(addr, buf)
    }
    fn leave(&mut self) -> Result<(), Self::Error> {
        self.leave()
    }
}
