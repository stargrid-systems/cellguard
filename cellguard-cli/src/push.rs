//! Building blocks for the push-image command.
//!
//! These are the pure pieces: key parsing and payload chunking. The
//! port-facing command lives in [`crate::commands::push_image`].

use std::error::Error;

/// One `BootData` frame ready to send.
pub struct DataFrame {
    /// Wire payload: a 4-byte little-endian start offset, then the chunk.
    pub bytes: Vec<u8>,
    /// The offset the device acknowledges after applying this frame.
    pub end_offset: u32,
}

/// Splits an image payload into [`DataFrame`]s.
///
/// Each frame carries at most `chunk_size` payload bytes behind its offset
/// header. An empty payload yields no frames. The caller must keep the
/// payload length within `u32`, which [`crate::commands::push_image`] checks
/// at entry.
///
/// # Examples
///
/// ```
/// use cellguard_cli::push::data_frames;
///
/// let frames: Vec<_> = data_frames(&[1, 2, 3], 2).collect();
/// assert_eq!(frames.len(), 2);
/// assert_eq!(frames[0].bytes, [0, 0, 0, 0, 1, 2]);
/// assert_eq!(frames[1].bytes, [2, 0, 0, 0, 3]);
/// assert_eq!(frames[1].end_offset, 3);
/// ```
///
/// # Panics
///
/// Panics if `chunk_size` is zero.
pub fn data_frames(payload: &[u8], chunk_size: usize) -> impl Iterator<Item = DataFrame> {
    let mut offset: u32 = 0;
    payload.chunks(chunk_size).map(move |chunk| {
        let mut bytes = Vec::with_capacity(4 + chunk.len());
        bytes.extend_from_slice(&offset.to_le_bytes());
        bytes.extend_from_slice(chunk);
        #[expect(
            clippy::cast_possible_truncation,
            reason = "the caller keeps the payload length within u32"
        )]
        let len = chunk.len() as u32;
        offset += len;
        DataFrame {
            bytes,
            end_offset: offset,
        }
    })
}

/// Parses a fleet HMAC key from 32 hex chars into 16 bytes.
///
/// Both hex letter cases are accepted.
///
/// # Examples
///
/// ```
/// let key = cellguard_cli::push::parse_key("000102030405060708090a0b0c0d0e0f").unwrap();
/// assert_eq!(key[0], 0x00);
/// assert_eq!(key[15], 0x0F);
/// ```
///
/// # Errors
///
/// Returns an error if the input is not exactly 32 chars or contains a
/// non-hex char.
pub fn parse_key(hex: &str) -> Result<[u8; 16], Box<dyn Error>> {
    if hex.len() != 32 {
        return Err(format!("key must be 32 hex chars (16 bytes), got {}", hex.len()).into());
    }
    let mut nibbles = hex.as_bytes().iter().map(|&c| hex_val(c));
    let mut key = [0u8; 16];
    for byte in &mut key {
        let hi = nibbles.next().ok_or("key too short")??;
        let lo = nibbles.next().ok_or("key too short")??;
        *byte = (hi << 4) | lo;
    }
    Ok(key)
}

fn hex_val(c: u8) -> Result<u8, Box<dyn Error>> {
    match c {
        b'0'..=b'9' => Ok(c - b'0'),
        b'a'..=b'f' => Ok(c - b'a' + 10),
        b'A'..=b'F' => Ok(c - b'A' + 10),
        _ => Err(format!("invalid hex char: {}", char::from(c)).into()),
    }
}

#[cfg(test)]
mod tests {
    use super::{data_frames, parse_key};

    #[test]
    fn parse_key_accepts_valid_hex() {
        let key = parse_key("00112233445566778899AaBbCcDdEeFf").unwrap();
        assert_eq!(
            key,
            [
                0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD,
                0xEE, 0xFF
            ]
        );
    }

    #[test]
    fn parse_key_rejects_odd_length() {
        assert!(parse_key(&"f".repeat(31)).is_err());
    }

    #[test]
    fn parse_key_rejects_wrong_length() {
        assert!(parse_key("").is_err());
        assert!(parse_key(&"f".repeat(30)).is_err());
        assert!(parse_key(&"f".repeat(64)).is_err());
    }

    #[test]
    fn parse_key_rejects_non_hex_chars() {
        assert!(parse_key(&"g".repeat(32)).is_err());
        assert!(parse_key(&format!("{}!", "f".repeat(31))).is_err());
    }

    #[test]
    fn data_frames_empty_payload_yields_nothing() {
        assert_eq!(data_frames(&[], 8).count(), 0);
    }

    #[test]
    fn data_frames_below_boundary_is_one_short_frame() {
        let frames: Vec<_> = data_frames(&[7; 5], 8).collect();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].bytes.len(), 4 + 5);
        assert_eq!(frames[0].bytes, [0, 0, 0, 0, 7, 7, 7, 7, 7]);
        assert_eq!(frames[0].end_offset, 5);
    }

    #[test]
    fn data_frames_at_boundary_is_one_full_frame() {
        let frames: Vec<_> = data_frames(&[7; 8], 8).collect();
        assert_eq!(frames.len(), 1);
        assert_eq!(frames[0].bytes.len(), 4 + 8);
        assert_eq!(frames[0].end_offset, 8);
    }

    #[test]
    fn data_frames_above_boundary_splits() {
        let payload: Vec<u8> = (0..9).collect();
        let frames: Vec<_> = data_frames(&payload, 8).collect();
        assert_eq!(frames.len(), 2);
        assert_eq!(frames[0].bytes.len(), 4 + 8);
        assert_eq!(frames[0].end_offset, 8);
        assert_eq!(frames[1].bytes, [8, 0, 0, 0, 8]);
        assert_eq!(frames[1].end_offset, 9);
    }

    #[test]
    fn data_frames_offsets_are_little_endian() {
        let payload = vec![0u8; 600];
        let frames: Vec<_> = data_frames(&payload, 256).collect();
        assert_eq!(frames.len(), 3);
        assert_eq!(frames[1].bytes[..4], 256u32.to_le_bytes());
        assert_eq!(frames[2].bytes[..4], 512u32.to_le_bytes());
        assert_eq!(frames[2].end_offset, 600);
    }
}
