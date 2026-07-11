//! An [`NvmWriter`] backed by the `updi` programmer.
//!
//! [`UpdiNvmWriter`] adapts the generic [`updi::Programmer`] to the
//! [`cellboot::io::NvmWriter`] trait, so [`crate::programmer::program`] can
//! drive an AVR Dx target over UPDI with no change. It erases each flash page
//! the first time a write touches it, then streams bytes straight to flash
//! (NVMCTRL v2 has no page buffer), so a sub-page or page-straddling chunk is
//! handled without buffering a whole page.

use cellboot::io::NvmWriter;
use updi::{PAGE_SIZE, ProgError, Programmer, UpdiLink};

/// An [`NvmWriter`] that programs an AVR Dx target over UPDI.
pub struct UpdiNvmWriter<L> {
    prog: Programmer<L>,
    erased_page: Option<u32>,
}

impl<L: UpdiLink> UpdiNvmWriter<L> {
    /// Wraps a UPDI transport.
    pub const fn new(link: L) -> Self {
        Self {
            prog: Programmer::new(link),
            erased_page: None,
        }
    }

    /// Releases the transport.
    pub fn free(self) -> L {
        self.prog.free()
    }
}

impl<L: UpdiLink> NvmWriter for UpdiNvmWriter<L> {
    type Error = ProgError<L::Error>;

    fn begin(&mut self) -> Result<(), Self::Error> {
        self.erased_page = None;
        self.prog.enter()
    }

    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error> {
        let mut addr = address;
        let mut rest = data;
        while !rest.is_empty() {
            let page = addr / PAGE_SIZE;
            if self.erased_page != Some(page) {
                self.prog.erase_flash_page(page.saturating_mul(PAGE_SIZE))?;
                self.erased_page = Some(page);
            }
            let page_end = page.saturating_add(1).saturating_mul(PAGE_SIZE);
            let room = usize::try_from(page_end.saturating_sub(addr)).unwrap_or(usize::MAX);
            let n = rest.len().min(room);
            let (chunk, tail) = rest.split_at(n);
            self.prog.write_flash(addr, chunk)?;
            addr = addr.saturating_add(u32::try_from(chunk.len()).unwrap_or(u32::MAX));
            rest = tail;
        }
        Ok(())
    }

    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.prog.read_flash(address, buf)
    }

    fn finish(&mut self) -> Result<(), Self::Error> {
        self.prog.leave()
    }
}

#[cfg(test)]
mod tests {
    use cellboot::io::NvmWriter;
    use updi::mock::MockTarget;

    use super::UpdiNvmWriter;

    fn ramp() -> [u8; 600] {
        core::array::from_fn(|i| u8::try_from(i % 251).unwrap())
    }

    #[test]
    fn single_write_straddles_page_boundary() {
        let mut w = UpdiNvmWriter::new(MockTarget::new());
        let data = ramp();
        w.begin().unwrap();
        // 600 bytes in one call cross the 512-byte page boundary: the adapter
        // must erase page 0 and page 1 and split the write.
        w.write(0, &data).unwrap();
        let mut back = [0u8; 600];
        w.read(0, &mut back).unwrap();
        w.finish().unwrap();
        assert_eq!(&back[..], &data[..]);
    }

    #[test]
    fn streamed_sub_page_chunks_program_correctly() {
        let mut w = UpdiNvmWriter::new(MockTarget::new());
        let data = ramp();
        w.begin().unwrap();
        for (i, chunk) in data.chunks(64).enumerate() {
            let offset = u32::try_from(i * 64).unwrap();
            w.write(offset, chunk).unwrap();
        }
        let mut back = [0u8; 600];
        w.read(0, &mut back).unwrap();
        w.finish().unwrap();
        assert_eq!(&back[..], &data[..]);
    }

    #[test]
    fn each_page_is_erased_once() {
        // Write page 0 in two chunks. The page must be erased on the first
        // chunk only, or the second chunk's data would be wiped.
        let mut w = UpdiNvmWriter::new(MockTarget::new());
        w.begin().unwrap();
        w.write(0, &[0x11; 8]).unwrap();
        w.write(8, &[0x22; 8]).unwrap();
        let mut back = [0u8; 16];
        w.read(0, &mut back).unwrap();
        w.finish().unwrap();
        assert_eq!(&back[..8], &[0x11; 8]);
        assert_eq!(&back[8..], &[0x22; 8]);
    }
}
