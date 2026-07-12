//! Power-on known-answer self-test for the crypto primitives.
//!
//! The firmware runs [`self_test`] once at boot, before it trusts any image or
//! key. It recomputes fixed NIST SHA-256 and RFC 4231 HMAC-SHA256 vectors on
//! the actual silicon and checks them against the published digests.
//!
//! This is not belt-and-braces. The AVR LLVM backend has miscompiled correct
//! Rust in the past (rust-lang/rust#109000), so a self-test on the target is
//! the only way to be sure the compiler that built this image did not silently
//! break the hash. A failure means the crypto cannot be trusted, so the caller
//! must refuse to run.

use hmac_sha256::{HMAC, Hash};

/// Which known-answer vector failed.
///
/// The caller can surface the specific variant (for example as a distinct
/// error code) to tell a hash break apart from an HMAC break.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KatError {
    /// A bare SHA-256 vector did not match.
    Sha256,
    /// An HMAC-SHA256 vector did not match.
    Hmac,
}

impl core::fmt::Display for KatError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let which = match self {
            Self::Sha256 => "SHA-256",
            Self::Hmac => "HMAC-SHA256",
        };
        write!(f, "{which} self-test failed")
    }
}

impl core::error::Error for KatError {}

/// SHA-256 of the empty string.
const SHA256_EMPTY: [u8; 32] = [
    0xe3, 0xb0, 0xc4, 0x42, 0x98, 0xfc, 0x1c, 0x14, 0x9a, 0xfb, 0xf4, 0xc8, 0x99, 0x6f, 0xb9, 0x24,
    0x27, 0xae, 0x41, 0xe4, 0x64, 0x9b, 0x93, 0x4c, 0xa4, 0x95, 0x99, 0x1b, 0x78, 0x52, 0xb8, 0x55,
];

/// SHA-256 of `"abc"`.
const SHA256_ABC: [u8; 32] = [
    0xba, 0x78, 0x16, 0xbf, 0x8f, 0x01, 0xcf, 0xea, 0x41, 0x41, 0x40, 0xde, 0x5d, 0xae, 0x22, 0x23,
    0xb0, 0x03, 0x61, 0xa3, 0x96, 0x17, 0x7a, 0x9c, 0xb4, 0x10, 0xff, 0x61, 0xf2, 0x00, 0x15, 0xad,
];

/// RFC 4231 test case 1: key of twenty `0x0b` bytes over `"Hi There"`.
const HMAC_CASE1: [u8; 32] = [
    0xb0, 0x34, 0x4c, 0x61, 0xd8, 0xdb, 0x38, 0x53, 0x5c, 0xa8, 0xaf, 0xce, 0xaf, 0x0b, 0xf1, 0x2b,
    0x88, 0x1d, 0xc2, 0x00, 0xc9, 0x83, 0x3d, 0xa7, 0x26, 0xe9, 0x37, 0x6c, 0x2e, 0x32, 0xcf, 0xf7,
];

/// RFC 4231 test case 2: key `"Jefe"` over `"what do ya want for nothing?"`.
const HMAC_CASE2: [u8; 32] = [
    0x5b, 0xdc, 0xc1, 0x46, 0xbf, 0x60, 0x75, 0x4e, 0x6a, 0x04, 0x24, 0x26, 0x08, 0x95, 0x75, 0xc7,
    0x5a, 0x00, 0x3f, 0x08, 0x9d, 0x27, 0x39, 0x83, 0x9d, 0xec, 0x58, 0xb9, 0x64, 0xec, 0x38, 0x43,
];

/// Runs the power-on crypto self-test.
///
/// Recomputes fixed SHA-256 and HMAC-SHA256 vectors on this device and checks
/// them against the published digests.
///
/// # Errors
///
/// Returns [`KatError`] identifying the first primitive whose output did not
/// match. On success the crypto is safe to use for this power cycle.
pub fn self_test() -> Result<(), KatError> {
    if Hash::hash(b"") != SHA256_EMPTY || Hash::hash(b"abc") != SHA256_ABC {
        return Err(KatError::Sha256);
    }

    if HMAC::mac(b"Hi There", [0x0b; 20]) != HMAC_CASE1
        || HMAC::mac(b"what do ya want for nothing?", b"Jefe") != HMAC_CASE2
    {
        return Err(KatError::Hmac);
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::self_test;

    #[test]
    fn self_test_passes_on_host() {
        assert_eq!(self_test(), Ok(()));
    }
}
