//! The streaming programmer engine.
//!
//! [`program`] streams a staged image from an [`ImageStore`] into a target
//! through an [`NvmWriter`]. Nothing larger than the caller's scratch buffer
//! is held in RAM. The HMAC is not checked here: authenticity was established
//! before staging. The payload CRC is checked twice, once against the staged
//! copy before the target is erased, and once against the written flash
//! before the target runs.

use crc::Crc32;

use crate::image::{HEADER_LEN, ImageHeader, ParseError};
use crate::io::{ImageStore, NvmWriter};

const HEADER_LEN_U32: u32 = 64;
const _: () = assert!(HEADER_LEN == HEADER_LEN_U32 as usize);

/// Programs a staged image into a target.
///
/// `image_offset` is where the image begins in `store`, `target_base` is
/// where the payload goes in the target's program memory, and `scratch` is
/// the streaming buffer. A size that divides the target page works well.
///
/// # Errors
///
/// Returns a [`ProgramError`] on any store or writer failure, an
/// unparseable header, a CRC mismatch, or an empty `scratch`.
pub fn program<S, W>(
    store: &mut S,
    writer: &mut W,
    image_offset: u32,
    target_base: u32,
    scratch: &mut [u8],
) -> Result<ImageHeader, ProgramError<S::Error, W::Error>>
where
    S: ImageStore,
    W: NvmWriter,
{
    // An empty scratch makes `chunk_len` return 0, so the loops below would
    // never advance.
    if scratch.is_empty() {
        return Err(ProgramError::EmptyScratch);
    }

    let mut header_bytes = [0u8; HEADER_LEN];
    store
        .read(image_offset, &mut header_bytes)
        .map_err(ProgramError::Store)?;
    let header = ImageHeader::parse(&header_bytes).map_err(ProgramError::Header)?;

    let payload_offset = image_offset.saturating_add(HEADER_LEN_U32);
    let payload_len = header.payload_len;

    let staged_crc = crc_over(
        |offset, buf| store.read(offset, buf).map_err(ProgramError::Store),
        payload_offset,
        payload_len,
        scratch,
    )?;
    if staged_crc != header.payload_crc32 {
        return Err(ProgramError::CorruptSource);
    }

    writer.begin().map_err(ProgramError::Nvm)?;
    let mut offset = 0u32;
    while offset < payload_len {
        let n = chunk_len(scratch.len(), payload_len - offset);
        let (buf, _) = scratch.split_at_mut(n);
        store
            .read(payload_offset.saturating_add(offset), buf)
            .map_err(ProgramError::Store)?;
        writer
            .write(target_base.saturating_add(offset), buf)
            .map_err(ProgramError::Nvm)?;
        offset = offset.saturating_add(advance(n));
    }

    let flash_crc = crc_over(
        |offset, buf| writer.read(offset, buf).map_err(ProgramError::Nvm),
        target_base,
        payload_len,
        scratch,
    )?;
    if flash_crc != header.payload_crc32 {
        return Err(ProgramError::VerifyFailed);
    }

    writer.finish().map_err(ProgramError::ReleaseFailed)?;
    Ok(header)
}

/// Whether a failed [`program`] attempt can succeed if retried.
///
/// A corrupt source or bad header never will, so the bootloader gives up on
/// them immediately.
#[must_use]
pub const fn retryable<S, N>(err: &ProgramError<S, N>) -> bool {
    !matches!(err, ProgramError::CorruptSource | ProgramError::Header(_))
}

fn chunk_len(capacity: usize, remaining: u32) -> usize {
    let cap = u32::try_from(capacity).unwrap_or(u32::MAX);
    usize::try_from(remaining.min(cap)).unwrap_or(capacity)
}

fn advance(n: usize) -> u32 {
    u32::try_from(n).unwrap_or(u32::MAX)
}

/// CRCs `len` bytes read through `read`, one scratch chunk at a time.
/// Shared by the store and flash passes so the loop is not monomorphized per
/// source.
fn crc_over<E>(
    mut read: impl FnMut(u32, &mut [u8]) -> Result<(), E>,
    base: u32,
    len: u32,
    scratch: &mut [u8],
) -> Result<u32, E> {
    let mut crc = Crc32::new();
    let mut done = 0u32;
    while done < len {
        let n = chunk_len(scratch.len(), len - done);
        let (buf, _) = scratch.split_at_mut(n);
        read(base.saturating_add(done), buf)?;
        crc.update(buf);
        done = done.saturating_add(advance(n));
    }
    Ok(crc.finalize())
}

/// An error from [`program`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[non_exhaustive]
pub enum ProgramError<S, N> {
    /// The caller passed an empty scratch buffer.
    EmptyScratch,
    /// Reading the staged image failed.
    Store(S),
    /// A programming operation failed.
    Nvm(N),
    /// The image header did not parse.
    Header(ParseError),
    /// The staged copy did not match its CRC (corrupt storage).
    CorruptSource,
    /// The written flash did not match its CRC (bad write).
    VerifyFailed,
    /// Release failed after a successful write and verify. Flash holds a
    /// valid image but the target was not released to run.
    ReleaseFailed(N),
}

#[cfg(test)]
mod tests {
    use super::{ProgramError, program};
    use crate::image::{HEADER_LEN, ImageHeader, ImageKind, Region};
    use crate::io::{ImageStore, NvmWriter};

    const STORE_CAP: usize = 1024;
    const FLASH_CAP: usize = 1024;

    struct MockStore {
        buf: [u8; STORE_CAP],
    }

    impl ImageStore for MockStore {
        type Error = ();

        fn capacity(&self) -> u32 {
            u32::try_from(STORE_CAP).unwrap()
        }

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()> {
            let start = usize::try_from(offset).unwrap();
            buf.copy_from_slice(&self.buf[start..start + buf.len()]);
            Ok(())
        }

        fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), ()> {
            let start = usize::try_from(offset).unwrap();
            self.buf[start..start + data.len()].copy_from_slice(data);
            Ok(())
        }
    }

    struct MockWriter {
        flash: [u8; FLASH_CAP],
        began: bool,
        finished: bool,
        /// If set, flip the byte written at this offset.
        corrupt_at: Option<usize>,
    }

    impl MockWriter {
        fn new() -> Self {
            Self {
                flash: [0x00; FLASH_CAP],
                began: false,
                finished: false,
                corrupt_at: None,
            }
        }
    }

    impl NvmWriter for MockWriter {
        type Error = ();

        fn begin(&mut self) -> Result<(), ()> {
            self.began = true;
            self.flash = [0xFF; FLASH_CAP];
            Ok(())
        }

        fn write(&mut self, address: u32, data: &[u8]) -> Result<(), ()> {
            let start = usize::try_from(address).unwrap();
            self.flash[start..start + data.len()].copy_from_slice(data);
            if let Some(at) = self.corrupt_at
                && (start..start + data.len()).contains(&at)
            {
                self.flash[at] ^= 0x01;
            }
            Ok(())
        }

        fn read(&mut self, address: u32, buf: &mut [u8]) -> Result<(), ()> {
            let start = usize::try_from(address).unwrap();
            buf.copy_from_slice(&self.flash[start..start + buf.len()]);
            Ok(())
        }

        fn finish(&mut self) -> Result<(), ()> {
            self.finished = true;
            Ok(())
        }
    }

    fn stage(store: &mut MockStore, payload: &[u8]) {
        let header = ImageHeader {
            kind: ImageKind::Application,
            region: Region::ApplicationCode,
            target_id: 1,
            fw_version: 1,
            payload_len: u32::try_from(payload.len()).unwrap(),
            payload_crc32: crc::checksum32(payload),
            hmac: [0u8; 32],
        };
        store.buf[..HEADER_LEN].copy_from_slice(&header.serialize());
        store.buf[HEADER_LEN..HEADER_LEN + payload.len()].copy_from_slice(payload);
    }

    fn ramp(len: usize) -> [u8; 300] {
        assert!(len <= 300);
        let mut out = [0u8; 300];
        for (i, b) in out.iter_mut().enumerate() {
            *b = u8::try_from(i % 251).unwrap();
        }
        out
    }

    #[test]
    fn programs_and_verifies() {
        let payload = ramp(300);
        let mut store = MockStore {
            buf: [0; STORE_CAP],
        };
        stage(&mut store, &payload);
        let mut writer = MockWriter::new();
        let mut scratch = [0u8; 64];

        let header = program(&mut store, &mut writer, 0, 0, &mut scratch).unwrap();
        assert_eq!(header.payload_len, 300);
        assert!(writer.began);
        assert!(writer.finished);
        assert_eq!(&writer.flash[..300], &payload[..]);
    }

    #[test]
    fn rejects_corrupt_source_before_touching_target() {
        let payload = ramp(128);
        let mut store = MockStore {
            buf: [0; STORE_CAP],
        };
        stage(&mut store, &payload);
        store.buf[HEADER_LEN + 10] ^= 0x01;
        let mut writer = MockWriter::new();
        let mut scratch = [0u8; 64];

        assert_eq!(
            program(&mut store, &mut writer, 0, 0, &mut scratch),
            Err(ProgramError::CorruptSource)
        );
        assert!(!writer.began);
        assert!(!writer.finished);
    }

    #[test]
    fn detects_bad_write_and_does_not_release() {
        let payload = ramp(128);
        let mut store = MockStore {
            buf: [0; STORE_CAP],
        };
        stage(&mut store, &payload);
        let mut writer = MockWriter::new();
        writer.corrupt_at = Some(50);
        let mut scratch = [0u8; 64];

        assert_eq!(
            program(&mut store, &mut writer, 0, 0, &mut scratch),
            Err(ProgramError::VerifyFailed)
        );
        assert!(writer.began);
        assert!(!writer.finished);
    }
}
