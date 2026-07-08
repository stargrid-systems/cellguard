//! HMAC-SHA256 (RFC 2104) used to authenticate firmware images.

use crate::mac::Mac;
use crate::sha256::{BLOCK_LEN, DIGEST_LEN, Sha256};

const IPAD: u8 = 0x36;
const OPAD: u8 = 0x5C;

/// Incremental HMAC-SHA256 computation.
///
/// Create it from the shared key with [`Hmac::new`], feed the message with
/// [`Hmac::update`], and read the 32-byte tag with [`Hmac::finalize`].
#[derive(Debug, Clone)]
pub struct Hmac {
    inner: Sha256,
    outer: Sha256,
}

impl Hmac {
    /// Creates an HMAC computation keyed with `key`.
    ///
    /// Keys longer than the SHA-256 block size are first hashed, as required by
    /// RFC 2104. Shorter keys are zero-padded.
    #[must_use]
    #[expect(
        clippy::indexing_slicing,
        reason = "the padded key block is fixed-size and all writes stay within \
                  it"
    )]
    pub fn new(key: &[u8]) -> Self {
        let mut block = [0u8; BLOCK_LEN];
        if key.len() > BLOCK_LEN {
            block[..DIGEST_LEN].copy_from_slice(&crate::sha256::digest(key));
        } else {
            block[..key.len()].copy_from_slice(key);
        }

        let mut inner = Sha256::new();
        let mut outer = Sha256::new();
        let mut inner_pad = [0u8; BLOCK_LEN];
        let mut outer_pad = [0u8; BLOCK_LEN];
        for (i, &byte) in block.iter().enumerate() {
            inner_pad[i] = byte ^ IPAD;
            outer_pad[i] = byte ^ OPAD;
        }
        inner.update(&inner_pad);
        outer.update(&outer_pad);

        Self { inner, outer }
    }

    /// Feeds message `data` into the computation.
    pub fn update(&mut self, data: &[u8]) {
        self.inner.update(data);
    }

    /// Consumes the computation and returns the 32-byte tag.
    #[must_use]
    pub fn finalize(mut self) -> [u8; DIGEST_LEN] {
        let inner = self.inner.finalize();
        self.outer.update(&inner);
        self.outer.finalize()
    }
}

impl Mac for Hmac {
    const TAG_LEN: usize = DIGEST_LEN;

    fn update(&mut self, data: &[u8]) {
        Self::update(self, data);
    }

    fn finalize(self) -> [u8; DIGEST_LEN] {
        Self::finalize(self)
    }
}

/// Computes the HMAC-SHA256 tag of `data` under `key` in one call.
#[must_use]
pub fn tag(key: &[u8], data: &[u8]) -> [u8; DIGEST_LEN] {
    let mut hmac = Hmac::new(key);
    hmac.update(data);
    hmac.finalize()
}

#[cfg(test)]
mod tests {
    use super::tag;

    #[expect(
        clippy::indexing_slicing,
        reason = "nibble indices are bounded to 0..16 and pairs come from chunks_exact_mut(2)"
    )]
    fn hex(bytes: &[u8; 32]) -> [u8; 64] {
        const LUT: &[u8; 16] = b"0123456789abcdef";
        let mut out = [0u8; 64];
        for (byte, pair) in bytes.iter().zip(out.chunks_exact_mut(2)) {
            pair[0] = LUT[usize::from(byte >> 4)];
            pair[1] = LUT[usize::from(byte & 0x0F)];
        }
        out
    }

    #[test]
    fn rfc4231_case1() {
        // Key = 20 * 0x0b, data = "Hi There".
        let key = [0x0Bu8; 20];
        assert_eq!(
            &hex(&tag(&key, b"Hi There")),
            b"b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"
        );
    }

    #[test]
    fn rfc4231_case2() {
        // Key = "Jefe", data = "what do ya want for nothing?".
        assert_eq!(
            &hex(&tag(b"Jefe", b"what do ya want for nothing?")),
            b"5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"
        );
    }

    #[test]
    fn long_key_is_hashed() {
        // Key = 131 * 0xaa, data = "Test Using Larger Than Block-Size Key -
        // Hash Key First" (RFC 4231 test case 6).
        let key = [0xAAu8; 131];
        assert_eq!(
            &hex(&tag(
                &key,
                b"Test Using Larger Than Block-Size Key - Hash Key First"
            )),
            b"60e431591ee0b67f0d8a26aacbf5b77f8e0bc6213728c5140546040f0ee37f54"
        );
    }
}
