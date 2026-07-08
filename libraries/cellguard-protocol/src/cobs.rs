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

    fn decode_frame<'a>(decoder: &'a mut Decoder<'_>, wire: &[u8]) -> &'a [u8] {
        let mut done = None;
        for &byte in wire {
            if let Some(len) = decoder.feed(byte).expect("decode should not fail") {
                done = Some(len);
            }
        }
        assert!(done.is_some(), "frame did not complete");
        decoder.data()
    }

    #[track_caller]
    fn assert_roundtrip(decoded: &[u8], encoded: &[u8]) {
        let mut wire = [0u8; 512];
        let n = encode(decoded, &mut wire);
        assert_eq!(&wire[..n], encoded, "encoding mismatch");

        let mut scratch = [0u8; 512];
        let mut dec = Decoder::new(&mut scratch);
        assert_eq!(decode_frame(&mut dec, encoded), decoded, "decoding mismatch");
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
        let mut decoder = Decoder::new(&mut scratch);
        assert_eq!(decode_frame(&mut decoder, &wire[..n]), &input);
    }
}
