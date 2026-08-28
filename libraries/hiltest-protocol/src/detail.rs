//! The `twi-scan` result detail: the list of acknowledged addresses.

/// First 7-bit address the `twi-scan` test probes.
pub const SCAN_FIRST: u8 = 0x08;

/// Last 7-bit address the `twi-scan` test probes (inclusive).
pub const SCAN_LAST: u8 = 0x77;

/// Capacity of an [`AckList`]: the whole probed range can ACK.
const CAP: usize = (SCAN_LAST - SCAN_FIRST + 1) as usize;

/// The acknowledged 7-bit addresses collected by the `twi-scan` test.
///
/// The wire form is the single-token result detail `acks=20,21,42`:
/// uppercase hex bytes without a `0x` prefix, comma-separated, and empty
/// after the `=` when no address acknowledged.
///
/// ```
/// use hiltest_protocol::AckList;
///
/// let mut acks = AckList::new();
/// assert!(acks.push(0x20));
/// assert!(acks.push(0x42));
/// assert_eq!(AckList::parse("acks=20,42"), Some(acks));
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AckList {
    // Slots past `len` stay zero, so the derived `PartialEq` is exact.
    addrs: [u8; CAP],
    len: u8,
}

impl AckList {
    /// Creates an empty list.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            addrs: [0; CAP],
            len: 0,
        }
    }

    /// Appends an address. Returns `false` when the list is full.
    pub fn push(&mut self, addr: u8) -> bool {
        let Some(slot) = self.addrs.get_mut(usize::from(self.len)) else {
            return false;
        };
        *slot = addr;
        self.len += 1;
        true
    }

    /// The collected addresses, in insertion order.
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        self.addrs.get(..usize::from(self.len)).unwrap_or(&[])
    }

    /// Parses the wire form. Returns [`None`] for anything the formatter
    /// does not produce, except that lowercase hex is tolerated.
    #[must_use]
    pub fn parse(detail: &str) -> Option<Self> {
        let rest = detail.strip_prefix("acks=")?;
        let mut list = Self::new();
        if rest.is_empty() {
            return Some(list);
        }
        for part in rest.split(',') {
            if part.is_empty() || part.len() > 2 || !part.bytes().all(|b| b.is_ascii_hexdigit()) {
                return None;
            }
            let addr = u8::from_str_radix(part, 16).ok()?;
            if !list.push(addr) {
                return None;
            }
        }
        Some(list)
    }
}

impl Default for AckList {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(feature = "ufmt")]
impl ufmt::uDisplay for AckList {
    fn fmt<W>(&self, f: &mut ufmt::Formatter<'_, W>) -> Result<(), W::Error>
    where
        W: ufmt::uWrite + ?Sized,
    {
        use crate::line::hex_char;

        ufmt::uwrite!(f, "acks=")?;
        let mut first = true;
        for &addr in self.as_slice() {
            if first {
                first = false;
            } else {
                ufmt::uwrite!(f, ",")?;
            }
            ufmt::uwrite!(f, "{}{}", hex_char(addr >> 4), hex_char(addr))?;
        }
        Ok(())
    }
}
