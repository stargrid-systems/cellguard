//! Reset controller (RSTCTRL).
//!
//! [`RstInstance`] reads reset-cause flags (`RSTFR`) and triggers a software
//! reset (`SWRR`). The register layout is shared across AVR128 DA/DB and
//! tinyAVR 0/1-series. `SWRR` is IOREG-protected.

use crate::clock::CcpUnlock;

/// Reset-cause flags from `RSTCTRL.RSTFR`. More than one may be set.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ResetFlags {
    bits: u8,
}

impl ResetFlags {
    /// No flags set.
    pub const EMPTY: Self = Self { bits: 0 };
    /// Power-on reset (PORF).
    pub const POWER_ON: Self = Self::from_bit(0);
    /// Brown-out reset (BORF).
    pub const BROWN_OUT: Self = Self::from_bit(1);
    /// External reset (EXTRF).
    pub const EXTERNAL: Self = Self::from_bit(2);
    /// Watchdog reset (WDRF).
    pub const WATCHDOG: Self = Self::from_bit(3);
    /// Software reset (SWRF). Set after [`RstInstance::software_reset`].
    pub const SOFTWARE: Self = Self::from_bit(4);
    /// UPDI reset (UPDIRF).
    pub const UPDI: Self = Self::from_bit(5);

    const fn from_bit(bit: u8) -> Self {
        Self { bits: 1 << bit }
    }

    /// Builds the flags from raw `RSTFR` bits, masking reserved bits.
    #[must_use]
    pub const fn from_bits_trimming(bits: u8) -> Self {
        Self { bits: bits & 0x3F }
    }

    /// The raw flag bits.
    #[must_use]
    pub const fn bits(self) -> u8 {
        self.bits
    }

    /// `true` if no flag is set.
    #[must_use]
    pub const fn is_empty(self) -> bool {
        self.bits == 0
    }

    /// `true` if all of `other` is set.
    #[must_use]
    pub const fn contains(self, other: Self) -> bool {
        self.bits & other.bits == other.bits
    }
}

/// A `RSTCTRL` peripheral. Implemented for each AVR128 and tinyAVR device. Not
/// for external use.
pub trait RstInstance {
    /// Reads the reset-cause flags.
    fn flags(&self) -> ResetFlags;
    /// Clears the given flags (write-`1`-to-clear).
    fn clear(&self, flags: ResetFlags);
    /// Triggers a software reset. Never returns.
    fn software_reset<C: CcpUnlock>(&self, cpu: &C) -> !;
}

/// Implements [`RstInstance`] for an `RSTCTRL`. `$swrr_bit` is the `SWRR` bit
/// name (`swrst` on AVR128, `swre` on tinyAVR).
macro_rules! impl_rst_instance {
    ($RSTCTRL:ty, $swrr_bit:ident) => {
        impl RstInstance for $RSTCTRL {
            #[inline(always)]
            fn flags(&self) -> ResetFlags {
                ResetFlags::from_bits_trimming(self.rstfr().read().bits())
            }

            #[inline(always)]
            fn clear(&self, flags: ResetFlags) {
                let b = flags.bits();
                self.rstfr().write(|w| {
                    w.porf()
                        .bit(b & 0x01 != 0)
                        .borf()
                        .bit(b & 0x02 != 0)
                        .extrf()
                        .bit(b & 0x04 != 0)
                        .wdrf()
                        .bit(b & 0x08 != 0)
                        .swrf()
                        .bit(b & 0x10 != 0)
                        .updirf()
                        .bit(b & 0x20 != 0)
                });
            }

            fn software_reset<C: CcpUnlock>(&self, cpu: &C) -> ! {
                avr_device::interrupt::disable();
                avr_device::interrupt::free(|_| {
                    cpu.unlock_ioreg();
                    self.swrr().write(|w| w.$swrr_bit().set_bit());
                });
                // Spin in case SWRR doesn't reset immediately.
                loop {
                    core::hint::spin_loop();
                }
            }
        }
    };
}

#[cfg(feature = "avr128db48")]
impl_rst_instance!(avr_device::avr128db48::RSTCTRL, swrst);
#[cfg(feature = "avr128db64")]
impl_rst_instance!(avr_device::avr128db64::RSTCTRL, swrst);
#[cfg(feature = "avr128da64")]
impl_rst_instance!(avr_device::avr128da64::RSTCTRL, swrst);
#[cfg(feature = "attiny406")]
impl_rst_instance!(avr_device::attiny406::RSTCTRL, swre);
#[cfg(feature = "attiny416")]
impl_rst_instance!(avr_device::attiny416::RSTCTRL, swre);

const _: () = {
    use crate::rstctrl::ResetFlags as F;
    assert!(F::POWER_ON.bits() == 0x01);
    assert!(F::BROWN_OUT.bits() == 0x02);
    assert!(F::EXTERNAL.bits() == 0x04);
    assert!(F::WATCHDOG.bits() == 0x08);
    assert!(F::SOFTWARE.bits() == 0x10);
    assert!(F::UPDI.bits() == 0x20);
    assert!(F::from_bits_trimming(0xC0).is_empty());

    let combined = F::from_bits_trimming(0x14);
    assert!(combined.contains(F::SOFTWARE));
    assert!(combined.contains(F::EXTERNAL));
    assert!(!combined.contains(F::WATCHDOG));
    assert!(!combined.contains(F::POWER_ON));
    assert!(!combined.is_empty());
    assert!(F::EMPTY.is_empty());
};
