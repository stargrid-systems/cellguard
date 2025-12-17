pub use self::decode::{DecodeError, Decoder};
pub use self::encode::Encoder;

mod decode;
mod encode;

#[cfg(test)]
mod tests {
    use super::*;

    #[track_caller]
    fn decode_full_slice<'a>(decoder: &'a mut Decoder, data: &[u8]) -> &'a [u8] {
        let (last, rest) = data.split_last().expect("data should not be empty");
        for &b in rest {
            let res = decoder.feed(b).ok().expect("feed should not error");
            assert!(res.is_none(), "expected intermediate feed to return None");
        }
        let res = decoder.feed(*last).ok().expect("feed should not error");
        res.expect("expected final feed to return Some");
        decoder.data()
    }

    #[track_caller]
    fn encode_full_slice(encoder: &mut Encoder<'_>, dest: &mut [u8]) -> usize {
        let mut pos = 0;
        loop {
            let byte = encoder.pull();
            match byte {
                Some(b) => {
                    dest[pos] = b;
                    pos += 1;
                }
                None => break,
            }
        }
        pos
    }

    #[track_caller]
    fn assert_roundtrip(decoded: &[u8], encoded: &[u8]) {
        let mut buf = [0u8; 512];

        let mut encoder = Encoder::new(decoded);
        let n = encode_full_slice(&mut encoder, &mut buf);
        let actual_encoded = &buf[..n];
        assert_eq!(actual_encoded, encoded, "encoding did not match expected");

        let mut decoder = Decoder::new_init(&mut buf);
        let actual_decoded = decode_full_slice(&mut decoder, encoded);
        assert_eq!(actual_decoded, decoded, "decoding did not match expected");
    }

    const fn generate_example_data(start: u8) -> [u8; 255] {
        let mut buf = [0u8; 255];
        let mut i = 0;
        while i < buf.len() {
            buf[i] = start.wrapping_add(i as u8);
            i += 1;
        }
        buf
    }

    #[test]
    fn roundtrip_example_1() {
        assert_roundtrip(&[0x00], &[0x01, 0x01, 0x00]);
    }

    #[test]
    fn roundtrip_example_2() {
        assert_roundtrip(&[0x00, 0x00], &[0x01, 0x01, 0x01, 0x00]);
    }

    #[test]
    fn roundtrip_example_3() {
        assert_roundtrip(&[0x00, 0x11, 0x00], &[0x01, 0x02, 0x11, 0x01, 0x00]);
    }

    #[test]
    fn roundtrip_example_5() {
        assert_roundtrip(
            &[0x11, 0x22, 0x33, 0x44],
            &[0x05, 0x11, 0x22, 0x33, 0x44, 0x00],
        );
    }

    #[test]
    fn roundtrip_example_11() {
        const ENCODED: &[u8] = b"\xfe\x03\x04\x05\x06\x07\x08\t\n\x0b\x0c\r\x0e\x0f\x10\x11\x12\x13\x14\x15\x16\x17\x18\x19\x1a\x1b\x1c\x1d\x1e\x1f !\"#$%&\'()*+,-./0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVWXYZ[\\]^_`abcdefghijklmnopqrstuvwxyz{|}~\x7f\x80\x81\x82\x83\x84\x85\x86\x87\x88\x89\x8a\x8b\x8c\x8d\x8e\x8f\x90\x91\x92\x93\x94\x95\x96\x97\x98\x99\x9a\x9b\x9c\x9d\x9e\x9f\xa0\xa1\xa2\xa3\xa4\xa5\xa6\xa7\xa8\xa9\xaa\xab\xac\xad\xae\xaf\xb0\xb1\xb2\xb3\xb4\xb5\xb6\xb7\xb8\xb9\xba\xbb\xbc\xbd\xbe\xbf\xc0\xc1\xc2\xc3\xc4\xc5\xc6\xc7\xc8\xc9\xca\xcb\xcc\xcd\xce\xcf\xd0\xd1\xd2\xd3\xd4\xd5\xd6\xd7\xd8\xd9\xda\xdb\xdc\xdd\xde\xdf\xe0\xe1\xe2\xe3\xe4\xe5\xe6\xe7\xe8\xe9\xea\xeb\xec\xed\xee\xef\xf0\xf1\xf2\xf3\xf4\xf5\xf6\xf7\xf8\xf9\xfa\xfb\xfc\xfd\xfe\xff\x02\x01\x00";
        assert_roundtrip(&generate_example_data(0x03), ENCODED);
    }
}
