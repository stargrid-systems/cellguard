//! Message authentication abstraction over HMAC-SHA256.
//!
//! The concrete hash and HMAC come from the [`hmac-sha256`] crate. This module
//! adds the [`Mac`] trait, so image verification is generic and testable, a
//! constant-time [`ct_eq`], and the known-answer tests.
//!
//! [`hmac-sha256`]: https://crates.io/crates/hmac-sha256

use hmac_sha256::HMAC;

/// A streaming message authentication code.
///
/// Implementors accumulate data with [`Mac::update`] and produce a fixed-length
/// tag with [`Mac::finalize`].
pub trait Mac {
    /// Length of the produced tag in bytes.
    const TAG_LEN: usize;

    /// Feeds `data` into the running computation.
    fn update(&mut self, data: &[u8]);

    /// Consumes the state and returns the authentication tag.
    fn finalize(self) -> [u8; 32];
}

impl Mac for HMAC {
    const TAG_LEN: usize = 32;

    fn update(&mut self, data: &[u8]) {
        Self::update(self, data);
    }

    fn finalize(self) -> [u8; 32] {
        Self::finalize(self)
    }
}

/// Domain-separation prefix for the key-replacement authentication tag.
///
/// Mixing this into the tag keeps a captured firmware-image HMAC from being
/// replayed as a key-replacement request.
pub const KEY_REPLACE_DOMAIN: &[u8] = b"CGKEYROT1";

/// Authenticates a key-replacement request in constant time.
///
/// Returns `true` when `tag` equals
/// `HMAC(current_key, KEY_REPLACE_DOMAIN || new_key)`. Only a holder of the
/// current key can produce a valid tag, so an unauthorized peer cannot rotate
/// the key.
#[must_use]
pub fn authenticate_key_replace(current_key: &[u8], new_key: &[u8], tag: &[u8; 32]) -> bool {
    let mut mac = HMAC::new(current_key);
    mac.update(KEY_REPLACE_DOMAIN);
    mac.update(new_key);
    ct_eq(&mac.finalize(), tag)
}

/// Compares two byte slices in constant time.
///
/// The running time depends only on the length of the inputs, not their
/// contents. Returns `false` immediately when the lengths differ.
#[must_use]
pub fn ct_eq(a: &[u8], b: &[u8]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut diff = 0u8;
    for (x, y) in a.iter().zip(b.iter()) {
        diff |= x ^ y;
    }
    diff == 0
}

#[cfg(test)]
mod tests {
    use hmac_sha256::{HMAC, Hash};

    use super::ct_eq;

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
    fn sha256_nist_vectors() {
        assert_eq!(
            &hex(&Hash::hash(b"")),
            b"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(
            &hex(&Hash::hash(b"abc")),
            b"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn hmac_rfc4231_vectors() {
        let key1 = [0x0Bu8; 20];
        assert_eq!(
            &hex(&HMAC::mac(b"Hi There", key1)),
            b"b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"
        );
        assert_eq!(
            &hex(&HMAC::mac(b"what do ya want for nothing?", b"Jefe")),
            b"5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"
        );
    }

    #[test]
    fn ct_eq_equal_and_unequal() {
        assert!(ct_eq(b"secret", b"secret"));
        assert!(!ct_eq(b"secret", b"secreT"));
        assert!(!ct_eq(b"secret", b"secre"));
    }
}
