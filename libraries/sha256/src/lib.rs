//! SHA-256 and HMAC-SHA256 for the `CellGuard` firmware.
//!
//! This crate is a thin wrapper over the [`hmac-sha256`] crate. It exists so the
//! rest of the firmware depends on a stable local API ([`Hmac`], [`Sha256`],
//! [`Mac`](mac::Mac), [`ct_eq`](mac::ct_eq)) rather than on the specific
//! upstream crate, and so this is the single place to keep the on-device
//! known-answer test.
//!
//! # AVR warning
//!
//! The LLVM AVR backend has a known miscompilation of rotate-and-add-heavy code
//! (rust-lang/rust#109000) that can silently corrupt hash output at some
//! optimization settings. This affects any SHA-256 implementation, crate or
//! hand-rolled. Any AVR build MUST run the known-answer tests below at its
//! production flags on real silicon before the output is trusted.
//!
//! [`hmac-sha256`]: https://crates.io/crates/hmac-sha256
#![no_std]
#![warn(missing_docs)]

pub use self::mac::{Mac, ct_eq};

pub mod mac;

/// Length of a SHA-256 digest in bytes.
pub const DIGEST_LEN: usize = 32;

/// Incremental SHA-256 hasher.
///
/// Feed data with [`Sha256::update`] and read the digest with
/// [`Sha256::finalize`].
#[derive(Clone)]
pub struct Sha256(hmac_sha256::Hash);

impl Default for Sha256 {
    fn default() -> Self {
        Self::new()
    }
}

impl Sha256 {
    /// Creates a hasher in its initial state.
    #[must_use]
    pub fn new() -> Self {
        Self(hmac_sha256::Hash::new())
    }

    /// Feeds `data` into the hasher.
    pub fn update(&mut self, data: &[u8]) {
        self.0.update(data);
    }

    /// Consumes the hasher and returns the final digest.
    #[must_use]
    pub fn finalize(self) -> [u8; DIGEST_LEN] {
        self.0.finalize()
    }
}

/// Computes the SHA-256 digest of `data` in one call.
#[must_use]
pub fn digest(data: &[u8]) -> [u8; DIGEST_LEN] {
    hmac_sha256::Hash::hash(data)
}

/// Incremental HMAC-SHA256 computation.
///
/// Create it from the shared key with [`Hmac::new`], feed the message with
/// [`Hmac::update`], and read the 32-byte tag with [`Hmac::finalize`].
#[derive(Clone)]
pub struct Hmac(hmac_sha256::HMAC);

impl Hmac {
    /// Creates an HMAC computation keyed with `key`.
    #[must_use]
    pub fn new(key: &[u8]) -> Self {
        Self(hmac_sha256::HMAC::new(key))
    }

    /// Feeds message `data` into the computation.
    pub fn update(&mut self, data: &[u8]) {
        self.0.update(data);
    }

    /// Consumes the computation and returns the 32-byte tag.
    #[must_use]
    pub fn finalize(self) -> [u8; DIGEST_LEN] {
        self.0.finalize()
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
    use super::{Hmac, Sha256, digest, tag};

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

    // Known-answer tests. On AVR these MUST also pass on real silicon at the
    // production optimization flags (see the crate-level AVR warning).

    #[test]
    fn sha256_nist_vectors() {
        assert_eq!(
            &hex(&digest(b"")),
            b"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(
            &hex(&digest(b"abc")),
            b"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
    }

    #[test]
    fn sha256_streaming_matches_oneshot() {
        let mut hasher = Sha256::new();
        for _ in 0..1000 {
            hasher.update(b"a");
        }
        assert_eq!(
            &hex(&hasher.finalize()),
            b"41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3"
        );
    }

    #[test]
    fn hmac_rfc4231_case1() {
        let key = [0x0Bu8; 20];
        assert_eq!(
            &hex(&tag(&key, b"Hi There")),
            b"b0344c61d8db38535ca8afceaf0bf12b881dc200c9833da726e9376c2e32cff7"
        );
    }

    #[test]
    fn hmac_rfc4231_case2() {
        assert_eq!(
            &hex(&tag(b"Jefe", b"what do ya want for nothing?")),
            b"5bdcc146bf60754e6a042426089575c75a003f089d2739839dec58b964ec3843"
        );
    }

    #[test]
    fn hmac_streaming_matches_oneshot() {
        let key = b"streaming-key";
        let mut hmac = Hmac::new(key);
        hmac.update(b"hello ");
        hmac.update(b"world");
        assert_eq!(hmac.finalize(), tag(key, b"hello world"));
    }
}
