//! Power-on known-answer self-test for the crypto primitives.
//!
//! [`self_test`] runs once at boot, before any image or key is trusted. The
//! AVR LLVM backend has miscompiled correct Rust in the past
//! (rust-lang/rust#109000), so the vectors must be recomputed on the actual
//! silicon. A failure means the crypto cannot be trusted.

use hmac_sha256::{HMAC, Hash};

/// Which known-answer vector failed.
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
    0xE3, 0xB0, 0xC4, 0x42, 0x98, 0xFC, 0x1C, 0x14, 0x9A, 0xFB, 0xF4, 0xC8, 0x99, 0x6F, 0xB9, 0x24,
    0x27, 0xAE, 0x41, 0xE4, 0x64, 0x9B, 0x93, 0x4C, 0xA4, 0x95, 0x99, 0x1B, 0x78, 0x52, 0xB8, 0x55,
];

/// SHA-256 of `"abc"`.
const SHA256_ABC: [u8; 32] = [
    0xBA, 0x78, 0x16, 0xBF, 0x8F, 0x01, 0xCF, 0xEA, 0x41, 0x41, 0x40, 0xDE, 0x5D, 0xAE, 0x22, 0x23,
    0xB0, 0x03, 0x61, 0xA3, 0x96, 0x17, 0x7A, 0x9C, 0xB4, 0x10, 0xFF, 0x61, 0xF2, 0x00, 0x15, 0xAD,
];

/// RFC 4231 test case 1: key of twenty `0x0b` bytes over `"Hi There"`.
const HMAC_CASE1: [u8; 32] = [
    0xB0, 0x34, 0x4C, 0x61, 0xD8, 0xDB, 0x38, 0x53, 0x5C, 0xA8, 0xAF, 0xCE, 0xAF, 0x0B, 0xF1, 0x2B,
    0x88, 0x1D, 0xC2, 0x00, 0xC9, 0x83, 0x3D, 0xA7, 0x26, 0xE9, 0x37, 0x6C, 0x2E, 0x32, 0xCF, 0xF7,
];

/// RFC 4231 test case 2: key `"Jefe"` over `"what do ya want for nothing?"`.
const HMAC_CASE2: [u8; 32] = [
    0x5B, 0xDC, 0xC1, 0x46, 0xBF, 0x60, 0x75, 0x4E, 0x6A, 0x04, 0x24, 0x26, 0x08, 0x95, 0x75, 0xC7,
    0x5A, 0x00, 0x3F, 0x08, 0x9D, 0x27, 0x39, 0x83, 0x9D, 0xEC, 0x58, 0xB9, 0x64, 0xEC, 0x38, 0x43,
];

/// Runs the power-on crypto self-test.
///
/// # Errors
///
/// Returns [`KatError`] identifying the first primitive whose output did not
/// match.
pub fn self_test() -> Result<(), KatError> {
    if Hash::hash(b"") != SHA256_EMPTY || Hash::hash(b"abc") != SHA256_ABC {
        return Err(KatError::Sha256);
    }

    if HMAC::mac(b"Hi There", [0x0B; 20]) != HMAC_CASE1
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
