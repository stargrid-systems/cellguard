//! Message authentication abstraction and constant-time comparison.

/// A streaming message authentication code.
///
/// Implementors accumulate data with [`Mac::update`] and produce a fixed-length
/// tag with [`Mac::finalize`]. The trait lets image verification be generic over
/// the concrete algorithm, which keeps the update logic testable with a mock.
pub trait Mac {
    /// Length of the produced tag in bytes.
    const TAG_LEN: usize;

    /// Feeds `data` into the running computation.
    fn update(&mut self, data: &[u8]);

    /// Consumes the state and returns the authentication tag.
    fn finalize(self) -> [u8; 32];
}

/// Compares two byte slices in constant time.
///
/// The running time depends only on the length of the inputs, not on their
/// contents, so it does not leak where a mismatching tag first differs. Returns
/// `false` immediately when the lengths differ.
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
    use super::ct_eq;

    #[test]
    fn equal_and_unequal() {
        assert!(ct_eq(b"secret", b"secret"));
        assert!(!ct_eq(b"secret", b"secreT"));
        assert!(!ct_eq(b"secret", b"secre"));
        assert!(ct_eq(b"", b""));
    }
}
