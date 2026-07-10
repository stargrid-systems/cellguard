//! Table-free bitwise CRC core, generic over the machine word.
//!
//! [`CrcCore`] holds the running state and runs the reflected bit loop once for
//! every input byte. [`Word`] carries the only thing that differs inside that
//! loop: the integer width and the reflected polynomial. Initial state and
//! final xor stay in the public wrappers, since those are the only per-variant
//! values a caller ever observes.

use core::ops::{BitAnd, BitXor, Shr};

mod sealed {
    pub trait Sealed {}
    impl Sealed for u16 {}
    impl Sealed for u32 {}
}

/// A machine word the bitwise CRC core runs on.
///
/// Sealed: implemented only for the integer widths this crate provides.
pub trait Word:
    Copy
    + self::sealed::Sealed
    + BitAnd<Output = Self>
    + BitXor<Output = Self>
    + Shr<u32, Output = Self>
{
    /// Reflected generator polynomial.
    const POLY: Self;

    /// Widens a data byte into the low bits.
    fn widen(byte: u8) -> Self;
    /// Two's-complement negation, wrapping instead of panicking on overflow.
    fn negate(self) -> Self;

    /// Folds one input byte in, then runs the eight reflected bit steps.
    #[must_use]
    fn update_byte(self, byte: u8) -> Self {
        let mut state = self ^ Self::widen(byte);
        for _ in 0..8 {
            // Branchless conditional xor: `mask` is all-ones when the low bit
            // is set and all-zeros otherwise.
            let mask = (state & Self::widen(1)).negate();
            state = (state >> 1) ^ (Self::POLY & mask);
        }
        state
    }
}

impl Word for u16 {
    const POLY: Self = 0xA001;
    fn widen(byte: u8) -> Self {
        Self::from(byte)
    }
    fn negate(self) -> Self {
        self.wrapping_neg()
    }
}

impl Word for u32 {
    const POLY: Self = 0xEDB8_8320;
    fn widen(byte: u8) -> Self {
        Self::from(byte)
    }
    fn negate(self) -> Self {
        self.wrapping_neg()
    }
}

/// Streaming state of a reflected bitwise CRC over the word `W`.
#[derive(Debug, Clone)]
pub struct CrcCore<W> {
    state: W,
}

impl<W: Copy> CrcCore<W> {
    /// Creates a core seeded with `init`.
    #[must_use]
    pub const fn new(init: W) -> Self {
        Self { state: init }
    }

    /// Returns the running state, before any final xor.
    #[must_use]
    pub const fn state(&self) -> W {
        self.state
    }
}

impl<W: Word> CrcCore<W> {
    /// Feeds `data` into the running state.
    pub fn update(&mut self, data: &[u8]) {
        for &byte in data {
            self.state = self.state.update_byte(byte);
        }
    }
}
