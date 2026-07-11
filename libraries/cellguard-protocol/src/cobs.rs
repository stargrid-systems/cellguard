//! Consistent Overhead Byte Stuffing (COBS) framing.
//!
//! COBS removes every zero byte from a frame and reserves `0x00` as the frame
//! delimiter. On a noisy multi-drop bus this gives unambiguous resynchronisation:
//! a receiver that loses sync only has to wait for the next `0x00`.
//!
//! Both halves are byte-at-a-time so they fit naturally in a UART interrupt:
//! [`Encoder::pull`] produces one wire byte per call, [`Decoder::feed`] consumes
//! one wire byte per call.

pub use self::decode::{DecodeError, Decoder};
pub use self::encode::Encoder;

mod decode;
mod encode;

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
    use super::{Decoder, Encoder};

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
        assert_roundtrip(&[0x11, 0x22, 0x33, 0x44], &[0x05, 0x11, 0x22, 0x33, 0x44, 0x00]);
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
}
