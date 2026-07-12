//! Consistent Overhead Byte Stuffing (COBS) framing.
//!
//! COBS removes every zero byte from a frame and reserves `0x00` as the frame
//! delimiter. On a noisy multi-drop bus this gives unambiguous
//! resynchronisation: a receiver that loses sync only has to wait for the next
//! `0x00`.
//!
//! Both halves are byte-at-a-time so they fit naturally in a UART interrupt:
//! [`Encoder::pull`] produces one wire byte per call, [`Decoder::feed`]
//! consumes one wire byte per call.

pub use self::decode::{DecodeError, Decoder};
pub use self::encode::Encoder;

mod decode;
mod encode;

/// Returns the maximum number of wire bytes that COBS encoding can produce for
/// `decoded_len` input bytes, including the frame terminator.
///
/// Use this to size the output buffer for [`encode_frame`] or [`Encoder`].
#[must_use]
pub const fn max_encoded_len(decoded_len: usize) -> usize {
    let blocks = if decoded_len <= 254 {
        1
    } else {
        decoded_len.div_ceil(254)
    };
    decoded_len + blocks + 1
}

/// Encodes `input` as a complete COBS frame, including the terminator, into
/// `out`.
///
/// Returns the number of bytes written, or `None` if `out` is too small. This
/// is the buffer-to-buffer convenience over [`Encoder`] for callers that have a
/// whole frame ready to send.
pub fn encode_frame(input: &[u8], out: &mut [u8]) -> Option<usize> {
    let mut encoder = Encoder::new(input);
    let mut pos = 0;
    while let Some(byte) = encoder.pull() {
        *out.get_mut(pos)? = byte;
        pos = pos.checked_add(1)?;
    }
    Some(pos)
}

#[cfg(test)]
mod tests {
    use super::{Decoder, Encoder, max_encoded_len};

    fn encode(input: &[u8], out: &mut [u8]) -> usize {
        let mut encoder = Encoder::new(input);
        let mut pos = 0;
        while let Some(byte) = encoder.pull() {
            out[pos] = byte;
            pos += 1;
        }
        pos
    }

    fn decode_frame(wire: &[u8], out: &mut [u8]) -> usize {
        let mut decoder = Decoder::new();
        let mut done = None;
        for &byte in wire {
            if let Some(len) = decoder.feed(byte, out).expect("decode should not fail") {
                done = Some(len);
            }
        }
        done.expect("frame did not complete")
    }

    #[track_caller]
    fn assert_roundtrip(decoded: &[u8], encoded: &[u8]) {
        let mut wire = [0u8; 512];
        let n = encode(decoded, &mut wire);
        assert_eq!(&wire[..n], encoded, "encoding mismatch");

        let mut scratch = [0u8; 512];
        let len = decode_frame(encoded, &mut scratch);
        assert_eq!(&scratch[..len], decoded, "decoding mismatch");
    }

    #[test]
    fn examples() {
        assert_roundtrip(&[0x00], &[0x01, 0x01, 0x00]);
        assert_roundtrip(&[0x00, 0x00], &[0x01, 0x01, 0x01, 0x00]);
        assert_roundtrip(&[0x00, 0x11, 0x00], &[0x01, 0x02, 0x11, 0x01, 0x00]);
        assert_roundtrip(
            &[0x11, 0x22, 0x33, 0x44],
            &[0x05, 0x11, 0x22, 0x33, 0x44, 0x00],
        );
    }

    #[test]
    fn long_block_without_zero() {
        // 254 non-zero bytes produce a single full block (code 0xFF) plus the
        // start of the next block.
        let input: [u8; 254] = core::array::from_fn(|i| u8::try_from(i % 255 + 1).unwrap_or(1));
        let mut wire = [0u8; 512];
        let n = encode(&input, &mut wire);
        let mut scratch = [0u8; 512];
        let len = decode_frame(&wire[..n], &mut scratch);
        assert_eq!(&scratch[..len], &input);
    }

    #[test]
    fn multi_block_no_spurious_completion() {
        // 255 non-zero bytes span two COBS blocks: a full 0xFF block (254
        // bytes) then a short block. The code byte that starts the second
        // block must not produce a frame-complete signal.
        let input: [u8; 255] = core::array::from_fn(|i| u8::try_from(i % 255 + 1).unwrap_or(1));
        let mut wire = [0u8; 512];
        let n = encode(&input, &mut wire);

        let mut decoder = Decoder::new();
        let mut scratch = [0u8; 512];
        let mut completions = [0usize; 4];
        let mut count = 0;

        for &byte in &wire[..n] {
            if let Some(len) = decoder
                .feed(byte, &mut scratch)
                .expect("decode should not fail")
            {
                completions[count] = len;
                count += 1;
            }
        }

        assert_eq!(count, 1, "expected exactly one frame-complete");
        assert_eq!(completions[0], 255);
        assert_eq!(&scratch[..255], &input);
    }

    #[test]
    fn max_encoded_len_never_underestimates() {
        for len in [0usize, 1, 2, 253, 254, 255, 256, 507, 508, 509, 1000] {
            let input: [u8; 1024] =
                core::array::from_fn(|i| u8::try_from(i % 255 + 1).unwrap_or(1));
            let bound = max_encoded_len(len);
            let mut wire = [0u8; 1024];
            let n = encode(&input[..len], &mut wire);
            assert!(n <= bound, "len {len}: encoded {n} exceeds bound {bound}");
        }
    }

    #[test]
    fn max_encoded_len_at_boundaries() {
        assert_eq!(max_encoded_len(0), 2);
        assert_eq!(max_encoded_len(1), 3);
        assert_eq!(max_encoded_len(254), 256);
        assert_eq!(max_encoded_len(255), 258);
    }
}
