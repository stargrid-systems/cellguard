use crate::command::HEADER_MAX;

/// CAT25 model information.
#[derive(Clone, Copy)]
#[must_use]
pub struct Model {
    page_size: u16,
    addr_bits: u8,
}

impl Model {
    /// Returns the number of bytes representing an address.
    #[must_use]
    pub const fn address_bytes(self) -> u8 {
        self.addr_bits.div_ceil(8)
    }

    /// Returns the total number of addressable (data) bytes for this model.
    #[must_use]
    pub const fn size(self) -> u32 {
        1 << self.addr_bits
    }

    /// Returns the page size in bytes.
    #[must_use]
    pub const fn page_size(self) -> u16 {
        self.page_size
    }

    /// Encodes an address into `buf` using the model's address byte count and
    /// returns how many bytes were written.
    ///
    /// Does not range-check the address and assumes `buf` is large enough.
    pub(crate) fn encode_address(self, buf: &mut [u8], address: u32) -> usize {
        let n = usize::from(self.address_bytes());
        debug_assert!(buf.len() >= n, "buffer too small for address");
        let bytes = address.to_be_bytes();
        let bytes = bytes.iter().skip(bytes.len() - n);
        buf.iter_mut().zip(bytes).for_each(|(dst, src)| *dst = *src);
        n
    }

    /// Encodes `opcode` followed by the model's address bytes into `buf`.
    ///
    /// Returns the populated prefix of `buf`.
    pub(crate) fn encode_header(
        self,
        buf: &mut [u8; HEADER_MAX],
        opcode: u8,
        address: u32,
    ) -> &mut [u8] {
        buf[0] = opcode;
        let n = self.encode_address(&mut buf[1..], address);
        #[expect(
            clippy::indexing_slicing,
            reason = "n is guaranteed to be in bounds of buf[1..]"
        )]
        &mut buf[..=n]
    }
}

/// 128-Kb EEPROM internally organized as 16Kx8 bits.
pub const CAT25128: Model = Model {
    page_size: 64,
    addr_bits: 14,
};

/// 1-Mb EEPROM internally organized as 128Kx8 bits.
pub const CAT25M01: Model = Model {
    page_size: 256,
    addr_bits: 17,
};

// `PageChunks` replaces the page modulo with a mask, so every model's page
// size must be a power of two.
const _: () = {
    assert!(CAT25128.page_size.is_power_of_two());
    assert!(CAT25M01.page_size.is_power_of_two());
};

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes_and_address_bytes() {
        assert_eq!(CAT25128.size(), 16_384);
        assert_eq!(CAT25128.address_bytes(), 2);
        assert_eq!(CAT25128.page_size(), 64);

        assert_eq!(CAT25M01.size(), 131_072);
        assert_eq!(CAT25M01.address_bytes(), 3);
        assert_eq!(CAT25M01.page_size(), 256);
    }

    #[test]
    fn encode_address_is_big_endian() {
        let mut buf = [0u8; 4];
        let n = CAT25128.encode_address(&mut buf, 0x1234);
        assert_eq!(n, 2);
        assert_eq!(buf.split_at(n).0, [0x12, 0x34]);

        let n = CAT25M01.encode_address(&mut buf, 0x01_2345);
        assert_eq!(n, 3);
        assert_eq!(buf.split_at(n).0, [0x01, 0x23, 0x45]);
    }
}
