//! Table-free SHA-256 (FIPS 180-4), HMAC-SHA256 (RFC 2104), and a small MAC
//! abstraction.
//!
//! Everything here is `#![no_std]`, dependency-free, allocation-free, and
//! streaming: feed data with `update` and read the result with `finalize`.
//!
//! The SHA-256 core lives at the crate root. HMAC-SHA256 is in [`hmac`] and the
//! [`Mac`](mac::Mac) abstraction plus constant-time comparison are in [`mac`].
//!
//! A note on AVR: the LLVM AVR backend has a known miscompilation of
//! rotate-and-add-heavy code (rust-lang/rust#109000) that can silently corrupt
//! hash output at some optimization settings. Any AVR build MUST run a
//! known-answer test against the vectors below at its production flags.
#![no_std]
#![warn(missing_docs)]

pub use self::hmac::Hmac;
pub use self::mac::{Mac, ct_eq};

pub mod hmac;
pub mod mac;

/// Length of a SHA-256 digest in bytes.
pub const DIGEST_LEN: usize = 32;

/// Length of a SHA-256 input block in bytes.
pub const BLOCK_LEN: usize = 64;

const INITIAL_STATE: [u32; 8] = [
    0x6A09_E667,
    0xBB67_AE85,
    0x3C6E_F372,
    0xA54F_F53A,
    0x510E_527F,
    0x9B05_688C,
    0x1F83_D9AB,
    0x5BE0_CD19,
];

const ROUND_CONSTANTS: [u32; 64] = [
    0x428A_2F98,
    0x7137_4491,
    0xB5C0_FBCF,
    0xE9B5_DBA5,
    0x3956_C25B,
    0x59F1_11F1,
    0x923F_82A4,
    0xAB1C_5ED5,
    0xD807_AA98,
    0x1283_5B01,
    0x2431_85BE,
    0x550C_7DC3,
    0x72BE_5D74,
    0x80DE_B1FE,
    0x9BDC_06A7,
    0xC19B_F174,
    0xE49B_69C1,
    0xEFBE_4786,
    0x0FC1_9DC6,
    0x240C_A1CC,
    0x2DE9_2C6F,
    0x4A74_84AA,
    0x5CB0_A9DC,
    0x76F9_88DA,
    0x983E_5152,
    0xA831_C66D,
    0xB003_27C8,
    0xBF59_7FC7,
    0xC6E0_0BF3,
    0xD5A7_9147,
    0x06CA_6351,
    0x1429_2967,
    0x27B7_0A85,
    0x2E1B_2138,
    0x4D2C_6DFC,
    0x5338_0D13,
    0x650A_7354,
    0x766A_0ABB,
    0x81C2_C92E,
    0x9272_2C85,
    0xA2BF_E8A1,
    0xA81A_664B,
    0xC24B_8B70,
    0xC76C_51A3,
    0xD192_E819,
    0xD699_0624,
    0xF40E_3585,
    0x106A_A070,
    0x19A4_C116,
    0x1E37_6C08,
    0x2748_774C,
    0x34B0_BCB5,
    0x391C_0CB3,
    0x4ED8_AA4A,
    0x5B9C_CA4F,
    0x682E_6FF3,
    0x748F_82EE,
    0x78A5_636F,
    0x84C8_7814,
    0x8CC7_0208,
    0x90BE_FFFA,
    0xA450_6CEB,
    0xBEF9_A3F7,
    0xC671_78F2,
];

/// Incremental SHA-256 hasher.
///
/// Feed data with [`Sha256::update`] and read the digest with
/// [`Sha256::finalize`]. Feeding in chunks yields the same digest as feeding
/// the whole input at once.
#[derive(Debug, Clone)]
pub struct Sha256 {
    state: [u32; 8],
    block: [u8; BLOCK_LEN],
    block_len: usize,
    total_len: u64,
}

impl Default for Sha256 {
    fn default() -> Self {
        Self::new()
    }
}

impl Sha256 {
    /// Creates a hasher in its initial state.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: INITIAL_STATE,
            block: [0; BLOCK_LEN],
            block_len: 0,
            total_len: 0,
        }
    }

    /// Feeds `data` into the hasher.
    #[expect(
        clippy::indexing_slicing,
        reason = "all indices are bounded by the fixed block length or by \
                  explicit length checks"
    )]
    pub fn update(&mut self, mut data: &[u8]) {
        self.total_len = self.total_len.wrapping_add(data.len() as u64);

        if self.block_len > 0 {
            let need = BLOCK_LEN - self.block_len;
            let take = need.min(data.len());
            self.block[self.block_len..self.block_len + take].copy_from_slice(&data[..take]);
            self.block_len += take;
            data = &data[take..];
            if self.block_len == BLOCK_LEN {
                let block = self.block;
                compress(&mut self.state, &block);
                self.block_len = 0;
            }
        }

        while data.len() >= BLOCK_LEN {
            let mut block = [0u8; BLOCK_LEN];
            block.copy_from_slice(&data[..BLOCK_LEN]);
            compress(&mut self.state, &block);
            data = &data[BLOCK_LEN..];
        }

        if !data.is_empty() {
            self.block[..data.len()].copy_from_slice(data);
            self.block_len = data.len();
        }
    }

    /// Consumes the hasher and returns the final digest.
    #[must_use]
    pub fn finalize(mut self) -> [u8; DIGEST_LEN] {
        let bit_len = self.total_len.wrapping_mul(8);

        self.update(&[0x80]);
        while self.block_len != 56 {
            self.update(&[0]);
        }
        self.update(&bit_len.to_be_bytes());

        let mut out = [0u8; DIGEST_LEN];
        for (word, chunk) in self.state.iter().zip(out.chunks_exact_mut(4)) {
            chunk.copy_from_slice(&word.to_be_bytes());
        }
        out
    }
}

#[expect(
    clippy::indexing_slicing,
    clippy::many_single_char_names,
    reason = "fixed-size schedule and working variables follow the FIPS 180-4 \
              naming and are indexed within range"
)]
fn compress(state: &mut [u32; 8], block: &[u8; BLOCK_LEN]) {
    let mut w = [0u32; 64];
    for (word, chunk) in w.iter_mut().zip(block.chunks_exact(4)) {
        *word = u32::from_be_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
    }
    for i in 16..64 {
        let s0 = w[i - 15].rotate_right(7) ^ w[i - 15].rotate_right(18) ^ (w[i - 15] >> 3);
        let s1 = w[i - 2].rotate_right(17) ^ w[i - 2].rotate_right(19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16]
            .wrapping_add(s0)
            .wrapping_add(w[i - 7])
            .wrapping_add(s1);
    }

    let mut a = state[0];
    let mut b = state[1];
    let mut c = state[2];
    let mut d = state[3];
    let mut e = state[4];
    let mut f = state[5];
    let mut g = state[6];
    let mut h = state[7];

    for i in 0..64 {
        let big_s1 = e.rotate_right(6) ^ e.rotate_right(11) ^ e.rotate_right(25);
        let ch = (e & f) ^ ((!e) & g);
        let t1 = h
            .wrapping_add(big_s1)
            .wrapping_add(ch)
            .wrapping_add(ROUND_CONSTANTS[i])
            .wrapping_add(w[i]);
        let big_s0 = a.rotate_right(2) ^ a.rotate_right(13) ^ a.rotate_right(22);
        let maj = (a & b) ^ (a & c) ^ (b & c);
        let t2 = big_s0.wrapping_add(maj);

        h = g;
        g = f;
        f = e;
        e = d.wrapping_add(t1);
        d = c;
        c = b;
        b = a;
        a = t1.wrapping_add(t2);
    }

    state[0] = state[0].wrapping_add(a);
    state[1] = state[1].wrapping_add(b);
    state[2] = state[2].wrapping_add(c);
    state[3] = state[3].wrapping_add(d);
    state[4] = state[4].wrapping_add(e);
    state[5] = state[5].wrapping_add(f);
    state[6] = state[6].wrapping_add(g);
    state[7] = state[7].wrapping_add(h);
}

/// Computes the SHA-256 digest of `data` in one call.
#[must_use]
pub fn digest(data: &[u8]) -> [u8; DIGEST_LEN] {
    let mut hasher = Sha256::new();
    hasher.update(data);
    hasher.finalize()
}

#[cfg(test)]
mod tests {
    use super::digest;

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

    #[test]
    fn nist_vectors() {
        assert_eq!(
            &hex(&digest(b"")),
            b"e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        );
        assert_eq!(
            &hex(&digest(b"abc")),
            b"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"
        );
        assert_eq!(
            &hex(&digest(
                b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"
            )),
            b"248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1"
        );
    }

    #[test]
    fn multi_block() {
        let mut hasher = super::Sha256::new();
        for _ in 0..1000 {
            hasher.update(b"a");
        }
        assert_eq!(
            &hex(&hasher.finalize()),
            b"41edece42d63e8d9bf515a9ba6932e1c20cbc9f5a5d134645adb5db1b9737ea3"
        );
    }
}
