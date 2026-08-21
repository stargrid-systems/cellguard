//! An [`NvmWriter`] backed by the `updi` programmer.
//!
//! [`UpdiNvmWriter`] adapts any [`FlashProg`] programmer to the shared
//! [`write_with_page_erase`] loop, so sub-page and page-straddling chunks
//! are written without buffering a whole page.

use cellboot::io::{NvmWriter, PagedFlash, write_with_page_erase};
use updi::FlashProg;

/// An [`NvmWriter`] that programs any UPDI flash target (AVR Dx or tinyAVR).
pub struct UpdiNvmWriter<P> {
    prog: P,
    erased_page: Option<u32>,
}

impl<P: FlashProg> UpdiNvmWriter<P> {
    /// Wraps a programmer.
    pub const fn new(prog: P) -> Self {
        Self {
            prog,
            erased_page: None,
        }
    }

    /// Releases the programmer.
    pub fn free(self) -> P {
        self.prog
    }
}

/// Adapts a `FlashProg` programmer to the cellboot `PagedFlash` seam
/// without cellboot depending on `updi`.
struct FlashProgAdapter<'a, P: FlashProg>(&'a mut P);

impl<P: FlashProg> PagedFlash for FlashProgAdapter<'_, P> {
    type Error = P::Error;

    fn erase_page(&mut self, page_base: u32) -> Result<(), Self::Error> {
        self.0.erase_page(page_base)
    }

    fn write_chunk(&mut self, addr: u32, chunk: &[u8]) -> Result<(), Self::Error> {
        self.0.write(addr, chunk)
    }
}

impl<P: FlashProg> NvmWriter for UpdiNvmWriter<P> {
    type Error = P::Error;

    fn begin(&mut self) -> Result<(), Self::Error> {
        self.erased_page = None;
        self.prog.enter()
    }

    fn write(&mut self, address: u32, data: &[u8]) -> Result<(), Self::Error> {
        // `self.prog` and `self.erased_page` are disjoint fields, so both can
        // be borrowed mutably in the same call.
        let mut adapter = FlashProgAdapter(&mut self.prog);
        write_with_page_erase(
            address,
            data,
            P::PAGE_SIZE,
            &mut self.erased_page,
            &mut adapter,
        )
    }

    fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), Self::Error> {
        self.prog.read(address, buf)
    }

    fn finish(&mut self) -> Result<(), Self::Error> {
        self.prog.leave()
    }
}

#[cfg(test)]
mod tests {
    use cellboot::io::NvmWriter;
    use updi::Programmer;
    use updi::mock::MockTarget;

    use super::UpdiNvmWriter;

    fn ramp() -> [u8; 600] {
        core::array::from_fn(|i| u8::try_from(i % 251).unwrap())
    }

    fn make() -> UpdiNvmWriter<Programmer<MockTarget>> {
        UpdiNvmWriter::new(Programmer::new(MockTarget::new()))
    }

    #[test]
    fn single_write_straddles_page_boundary() {
        let mut w = make();
        let data = ramp();
        w.begin().unwrap();
        // 600 bytes in one call cross the 512-byte page boundary.
        w.write(0, &data).unwrap();
        let mut back = [0u8; 600];
        w.read(0, &mut back).unwrap();
        w.finish().unwrap();
        assert_eq!(&back[..], &data[..]);
    }

    #[test]
    fn streamed_sub_page_chunks_program_correctly() {
        let mut w = make();
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
        let mut w = make();
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
