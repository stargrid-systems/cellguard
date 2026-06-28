use core::mem;

/// Block protection bits for [`Status::block_protection`].
#[derive(Clone, Copy)]
#[must_use]
#[repr(u8)]
pub enum BlockProtection {
    /// No part of the array is protected.
    None = 0b00,
    /// The upper quarter of the array is protected.
    Quarter = 0b01,
    /// The upper half of the array is protected.
    Half = 0b10,
    /// The entire array is protected.
    All = 0b11,
}

/// Status register.
///
/// Returned by reads of the device. The driver builds the payloads for status
/// register writes internally, so this type is read-only to callers.
#[derive(Clone, Copy)]
#[must_use]
#[repr(transparent)]
pub struct Status(u8);

impl Status {
    const RDY_INV_MASK: u8 = 0b0000_0001;
    const WEL_MASK: u8 = 0b0000_0010;
    const BP_MASK: u8 = 0b0000_1100;
    const LIP_MASK: u8 = 0b0001_0000;
    const IPL_MASK: u8 = 0b0100_0000;
    const WPEN_MASK: u8 = 0b1000_0000;

    /// Bits that the WRSR command is allowed to write.
    const WRITABLE_MASK: u8 = Self::BP_MASK | Self::LIP_MASK | Self::IPL_MASK | Self::WPEN_MASK;

    /// Wraps a raw status register byte read from the device.
    pub(crate) const fn from_bits(bits: u8) -> Self {
        Self(bits)
    }

    /// Returns the raw status register byte.
    pub(crate) const fn bits(self) -> u8 {
        self.0
    }

    /// Masks to the bits that the WRSR command can write.
    ///
    /// Used to carry the current writable bits into a read-modify-write so a
    /// status change does not clobber the bits it is not touching.
    pub(crate) const fn writable(self) -> Self {
        Self(self.0 & Self::WRITABLE_MASK)
    }

    /// Returns true if the device is ready for a new operation.
    #[must_use]
    pub const fn ready(self) -> bool {
        self.0 & Self::RDY_INV_MASK == 0
    }

    /// Returns true if the device is write enabled.
    #[must_use]
    pub const fn write_enabled(self) -> bool {
        self.0 & Self::WEL_MASK != 0
    }

    /// Returns the block protection bits.
    pub const fn block_protection(self) -> BlockProtection {
        let raw = (self.0 & Self::BP_MASK) >> const { Self::BP_MASK.trailing_zeros() };
        // SAFETY: `BlockProtection` covers all four combinations of two bits.
        unsafe { mem::transmute(raw) }
    }

    /// Returns true if the identification page latch is set.
    ///
    /// While set, the read and write commands target the identification page
    /// instead of the main array. The device clears this bit after the next
    /// read or write.
    #[must_use]
    pub const fn id_page_latch(self) -> bool {
        self.0 & Self::IPL_MASK != 0
    }

    /// Returns true if the identification page is permanently locked.
    #[must_use]
    pub const fn id_page_locked(self) -> bool {
        self.0 & Self::LIP_MASK != 0
    }

    /// Returns true if the write protection is enabled.
    #[must_use]
    pub const fn write_protect_enabled(self) -> bool {
        self.0 & Self::WPEN_MASK != 0
    }

    const fn with_bit(mut self, mask: u8, enable: bool) -> Self {
        if enable {
            self.0 |= mask;
        } else {
            self.0 &= !mask;
        }
        self
    }

    /// Sets the block protection bits.
    pub(crate) const fn with_block_protection(self, protection: BlockProtection) -> Self {
        let bits = (protection as u8) << Self::BP_MASK.trailing_zeros();
        Self((self.0 & !Self::BP_MASK) | bits)
    }

    /// Sets the identification page latch bit.
    pub(crate) const fn with_id_page_latch(self, enable: bool) -> Self {
        self.with_bit(Self::IPL_MASK, enable)
    }

    /// Sets the permanent identification page lock bit.
    ///
    /// Locking the identification page is irreversible.
    pub(crate) const fn with_lock_id_page(self, enable: bool) -> Self {
        self.with_bit(Self::LIP_MASK, enable)
    }

    /// Sets the write protect enable bit.
    pub(crate) const fn with_write_protect_enabled(self, enable: bool) -> Self {
        self.with_bit(Self::WPEN_MASK, enable)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn decodes_each_field() {
        assert!(Status(0b0000_0000).ready());
        assert!(!Status(0b0000_0001).ready());
        assert!(Status(0b0000_0010).write_enabled());
        assert!(matches!(
            Status(0b0000_1000).block_protection(),
            BlockProtection::Half
        ));
        assert!(matches!(
            Status(0b0000_1100).block_protection(),
            BlockProtection::All
        ));
        assert!(Status(0b0001_0000).id_page_locked());
        assert!(!Status(0b0001_0000).id_page_latch());
        assert!(Status(0b0100_0000).id_page_latch());
        assert!(!Status(0b0100_0000).id_page_locked());
        assert!(Status(0b1000_0000).write_protect_enabled());
    }

    #[test]
    fn builders_set_only_target_bits() {
        let zero = Status::from_bits(0);
        assert_eq!(zero.with_id_page_latch(true).0, 0b0100_0000);
        assert_eq!(zero.with_lock_id_page(true).0, 0b0001_0000);
        assert_eq!(zero.with_write_protect_enabled(true).0, 0b1000_0000);
        assert_eq!(
            zero.with_block_protection(BlockProtection::All).0,
            0b0000_1100
        );
        assert_eq!(
            zero.with_block_protection(BlockProtection::Quarter).0,
            0b0000_0100
        );
    }

    #[test]
    fn writable_masks_status_and_control_bits() {
        // RDY, WEL, and the reserved bit are not writable.
        assert_eq!(Status::from_bits(0b1111_1111).writable().0, 0b1101_1100);
    }
}
