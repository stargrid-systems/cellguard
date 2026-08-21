//! The programmer's bus-facing driver.
//!
//! The programmer's single USART is shared between the UART command link
//! and the UPDI link through a mux, so the firmware owns the USART and
//! lends the [`NvmWriter`] to the supervisor only for a flash. Decode,
//! program, and reply are therefore separate methods ([`Supervisor::decode`],
//! [`Supervisor::program`], [`Supervisor::reply`]), with
//! [`Supervisor::service`] doing all three in one call for hosts and tests.

use cellboot::io::{ImageStore, NvmWriter};
use cellboot::programmer::{ProgramError, program};
use cellguard_protocol::{
    Decoder, HEADER_LEN, Kind, PAYLOAD_CRC_LEN, Packet, ProgSource, ProgStatus, encode_frame,
    max_encoded_len,
};

const SCRATCH: usize = 64;

const RESULT_FRAME: usize = HEADER_LEN + 1 + PAYLOAD_CRC_LEN;

/// Computed from the shared helper so it cannot drift.
const RESULT_WIRE: usize = max_encoded_len(RESULT_FRAME);

/// Where a source image sits in the store and where it maps in the target.
#[derive(Debug, Clone, Copy)]
pub struct SourceSlot {
    /// Byte offset of the image (header then payload) within the store.
    pub image_offset: u32,
    /// Base address the payload is written to in the target's program memory.
    pub target_base: u32,
}

/// Maps each [`ProgSource`] to its slot.
#[derive(Debug, Clone, Copy)]
pub struct ProgLayout {
    /// Slot for [`ProgSource::AppStaged`].
    pub app: SourceSlot,
    /// Slot for [`ProgSource::BootloaderStaged`].
    pub bootloader: SourceSlot,
    /// Slot for [`ProgSource::CellagentAppStaged`].
    pub cellagent: SourceSlot,
}

impl ProgLayout {
    const fn slot(&self, source: ProgSource) -> SourceSlot {
        match source {
            ProgSource::AppStaged => self.app,
            ProgSource::BootloaderStaged => self.bootloader,
            ProgSource::CellagentAppStaged => self.cellagent,
        }
    }
}

/// The programmer's bus driver.
///
/// `RX` sizes the receive buffer. The [`NvmWriter`] is supplied per flash
/// so the firmware can share one USART between the command link and UPDI.
pub struct Supervisor<S, const RX: usize> {
    store: S,
    layout: ProgLayout,
    id: u8,
    decoder: Decoder,
    rx: [u8; RX],
    tx: [u8; RESULT_WIRE],
    scratch: [u8; SCRATCH],
}

impl<S: ImageStore, const RX: usize> Supervisor<S, RX> {
    /// Creates a supervisor for node `id`.
    pub const fn new(store: S, layout: ProgLayout, id: u8) -> Self {
        Self {
            store,
            layout,
            id,
            decoder: Decoder::new(),
            rx: [0; RX],
            tx: [0; RESULT_WIRE],
            scratch: [0; SCRATCH],
        }
    }

    /// Feeds one received wire byte from the cellcore link. Returns the
    /// source on a complete, valid `ProgProgram` addressed to this node.
    pub fn decode(&mut self, byte: u8) -> Option<ProgSource> {
        let Ok(Some(len)) = self.decoder.feed(byte, &mut self.rx) else {
            return None;
        };
        let frame = self.rx.get(..len)?;
        let packet = Packet::parse(frame).ok()?;
        if packet.id != self.id || packet.kind != Kind::ProgProgram {
            return None;
        }
        ProgSource::from_code(*packet.payload.first()?)
    }

    /// Flashes the staged image for `source` through `writer`.
    ///
    /// The writer is borrowed so the firmware can lend the shared
    /// USART/UPDI link for just this call.
    pub fn program<W: NvmWriter>(&mut self, source: ProgSource, writer: &mut W) -> ProgStatus {
        let slot = self.layout.slot(source);
        match program(
            &mut self.store,
            writer,
            slot.image_offset,
            slot.target_base,
            &mut self.scratch,
        ) {
            Ok(_) => ProgStatus::Ok,
            Err(ProgramError::CorruptSource) => ProgStatus::CorruptSource,
            Err(ProgramError::VerifyFailed) => ProgStatus::VerifyFailed,
            Err(ProgramError::ReleaseFailed(_)) => ProgStatus::OkReleaseFailed,
            // Store, Nvm, Header, and any future variant.
            Err(_) => ProgStatus::Failed,
        }
    }

    /// Encodes a `ProgResult(status)` reply into the internal transmit
    /// buffer and returns it.
    #[must_use]
    pub fn reply(&mut self, status: ProgStatus) -> Option<&[u8]> {
        let mut raw = [0u8; RESULT_FRAME];
        let raw_len =
            Packet::write(self.id, Kind::ProgResult, &[status.to_code()], &mut raw).ok()?;
        let wire_len = encode_frame(raw.get(..raw_len)?, &mut self.tx)?;
        self.tx.get(..wire_len)
    }

    /// Decodes, programs, and replies in one call, for hosts and tests where
    /// the writer is always available.
    pub fn service<W: NvmWriter>(&mut self, byte: u8, writer: &mut W) -> Option<&[u8]> {
        let source = self.decode(byte)?;
        let status = self.program(source, writer);
        self.reply(status)
    }
}

#[cfg(test)]
mod tests {
    use cellboot::image::{HEADER_LEN, ImageHeader, ImageKind, Region};
    use cellboot::io::{ImageStore, NvmWriter};
    use cellguard_protocol::{Decoder, Kind, Packet, ProgSource, ProgStatus, encode_frame};

    use super::{ProgLayout, SourceSlot, Supervisor};

    const STORE_CAP: usize = 2048;
    const FLASH_CAP: usize = 1024;
    const NODE: u8 = 3;
    const APP_OFFSET: u32 = 0;

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
        finished: bool,
    }

    impl NvmWriter for MockWriter {
        type Error = ();

        fn begin(&mut self) -> Result<(), ()> {
            self.flash = [0xFF; FLASH_CAP];
            Ok(())
        }

        fn write(&mut self, address: u32, data: &[u8]) -> Result<(), ()> {
            let start = usize::try_from(address).unwrap();
            self.flash[start..start + data.len()].copy_from_slice(data);
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

    fn stage(store: &mut MockStore, offset: u32, payload: &[u8]) {
        let header = ImageHeader {
            kind: ImageKind::Application,
            region: Region::ApplicationCode,
            target_id: 1,
            fw_version: 1,
            payload_len: u32::try_from(payload.len()).unwrap(),
            payload_crc32: crc::checksum32(payload),
            hmac: [0u8; 32],
        };
        let base = usize::try_from(offset).unwrap();
        store.buf[base..base + HEADER_LEN].copy_from_slice(&header.serialize());
        store.buf[base + HEADER_LEN..base + HEADER_LEN + payload.len()].copy_from_slice(payload);
    }

    fn layout() -> ProgLayout {
        ProgLayout {
            app: SourceSlot {
                image_offset: APP_OFFSET,
                target_base: 0,
            },
            bootloader: SourceSlot {
                image_offset: 512,
                target_base: 0,
            },
            cellagent: SourceSlot {
                image_offset: 1024,
                target_base: 0x8000,
            },
        }
    }

    fn make() -> Supervisor<MockStore, 64> {
        let store = MockStore {
            buf: [0; STORE_CAP],
        };
        Supervisor::new(store, layout(), NODE)
    }

    fn decode_response(frame: &[u8]) -> (Kind, u8) {
        let mut scratch = [0u8; 32];
        let mut decoder = Decoder::new();
        let mut done = None;
        for &byte in frame {
            if let Some(m) = decoder.feed(byte, &mut scratch).unwrap() {
                done = Some(m);
            }
        }
        let packet = Packet::parse(&scratch[..done.unwrap()]).unwrap();
        (packet.kind, *packet.payload.first().unwrap_or(&0))
    }

    #[test]
    fn services_a_request_packet() {
        let mut sup = make();
        let mut writer = MockWriter {
            flash: [0; FLASH_CAP],
            finished: false,
        };
        let payload: [u8; 200] = core::array::from_fn(|i| u8::try_from(i % 251).unwrap());
        stage(&mut sup.store, APP_OFFSET, &payload);

        let mut raw = [0u8; 32];
        let raw_len = Packet::write(NODE, Kind::ProgProgram, &[0], &mut raw).unwrap();
        let mut wire = [0u8; 48];
        let wire_len = encode_frame(&raw[..raw_len], &mut wire).unwrap();

        let mut response = None;
        for &byte in &wire[..wire_len] {
            if let Some(frame) = sup.service(byte, &mut writer) {
                let mut copy = [0u8; 32];
                copy[..frame.len()].copy_from_slice(frame);
                response = Some((copy, frame.len()));
            }
        }
        let (wire_resp, n) = response.expect("expected a ProgResult");

        let (kind, code) = decode_response(&wire_resp[..n]);
        assert_eq!(kind, Kind::ProgResult);
        assert_eq!(code, ProgStatus::Ok.to_code());
    }

    #[test]
    fn decode_then_program_then_reply_round_trips() {
        let mut sup = make();
        let mut writer = MockWriter {
            flash: [0; FLASH_CAP],
            finished: false,
        };
        let payload: [u8; 128] = core::array::from_fn(|i| u8::try_from(i).unwrap());
        stage(&mut sup.store, APP_OFFSET, &payload);

        let mut raw = [0u8; 32];
        let raw_len = Packet::write(NODE, Kind::ProgProgram, &[0], &mut raw).unwrap();
        let mut wire = [0u8; 48];
        let wire_len = encode_frame(&raw[..raw_len], &mut wire).unwrap();

        let mut source = None;
        for &byte in &wire[..wire_len] {
            if let Some(s) = sup.decode(byte) {
                source = Some(s);
            }
        }
        assert_eq!(source, Some(ProgSource::AppStaged));

        let status = sup.program(ProgSource::AppStaged, &mut writer);
        assert_eq!(status, ProgStatus::Ok);
        assert!(writer.finished);
        assert_eq!(&writer.flash[..128], &payload[..]);

        let frame = sup.reply(status).unwrap();
        let (kind, code) = decode_response(frame);
        assert_eq!(kind, Kind::ProgResult);
        assert_eq!(code, ProgStatus::Ok.to_code());
    }

    #[test]
    fn corrupt_source_reports_failure() {
        let mut sup = make();
        let mut writer = MockWriter {
            flash: [0; FLASH_CAP],
            finished: false,
        };
        let payload: [u8; 128] = core::array::from_fn(|i| u8::try_from(i).unwrap());
        stage(&mut sup.store, APP_OFFSET, &payload);
        sup.store.buf[usize::try_from(APP_OFFSET).unwrap() + HEADER_LEN + 5] ^= 0x01;

        let status = sup.program(ProgSource::AppStaged, &mut writer);
        assert_eq!(status, ProgStatus::CorruptSource);
        assert!(!writer.finished);
    }
}
