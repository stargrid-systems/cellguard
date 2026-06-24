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

    /// Returns the identification page size in bytes.
    ///
    /// On the supported parts the identification page is exactly one regular
    /// page.
    #[must_use]
    pub const fn id_page_size(self) -> u16 {
        self.page_size
    }

    /// Encodes an address into the buffer using the correct number of bytes for
    /// the model.
    ///
    /// This does not validate that the address is in range (that depends on
    /// whether the access targets the id page or the main array) and assumes
    /// the buffer is large enough to hold the address bytes.
    ///
    /// Returns the number of bytes written to the buffer.
    pub(crate) fn encode_address(self, buf: &mut [u8], address: u32) -> usize {
        let n = usize::from(self.address_bytes());
        let bytes = address.to_be_bytes();
        // The model uses the low `n` bytes, which are the last `n` big-endian
        // bytes of the address.
        for (dst, src) in buf.iter_mut().zip(bytes.iter().skip(bytes.len() - n)) {
            *dst = *src;
        }
        n
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes_and_address_bytes() {
        assert_eq!(CAT25128.size(), 16_384);
        assert_eq!(CAT25128.address_bytes(), 2);
        assert_eq!(CAT25128.page_size(), 64);
        assert_eq!(CAT25128.id_page_size(), 64);

        assert_eq!(CAT25M01.size(), 131_072);
        assert_eq!(CAT25M01.address_bytes(), 3);
        assert_eq!(CAT25M01.page_size(), 256);
        assert_eq!(CAT25M01.id_page_size(), 256);
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
