//! The `cellprog` supervisor: the programmer's bus-facing driver.
//!
//! [`Supervisor`] owns the [`ImageStore`] and [`NvmWriter`] and answers the
//! main MCU over the local `UART_PROG` link, which speaks the same
//! [`cellguard_protocol`] as the field bus. On a [`Kind::ProgProgram`] request
//! it runs [`crate::programmer::program`] for the selected source and replies
//! with a [`Kind::ProgResult`].
//!
//! The `TINY_ALIVE` heartbeat is a hardware line, not a packet: the firmware
//! polls it and calls [`Supervisor::recover`] when the main MCU goes silent,
//! which reprograms it from the golden image.

use cellboot::io::{ImageStore, NvmWriter};
use cellguard_protocol::{
    Decoder, HEADER_LEN, Kind, PAYLOAD_CRC_LEN, Packet, ProgSource, ProgStatus, encode_frame,
};

use crate::programmer::{ProgramError, program};

/// Streaming scratch buffer size the programmer uses.
const SCRATCH: usize = 64;

/// A `ProgResult` frame is a header, a one-byte status, and the payload CRC.
const RESULT_FRAME: usize = HEADER_LEN + 1 + PAYLOAD_CRC_LEN;

/// Worst-case COBS-encoded size of a result frame, including the terminator.
const RESULT_WIRE: usize = RESULT_FRAME + RESULT_FRAME.div_ceil(254) + 1;

/// Where each source image sits in the store and where it maps in the target.
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
    /// Slot for [`ProgSource::Golden`].
    pub golden: SourceSlot,
}

impl ProgLayout {
    const fn slot(&self, source: ProgSource) -> SourceSlot {
        match source {
            ProgSource::AppStaged => self.app,
            ProgSource::BootloaderStaged => self.bootloader,
            ProgSource::Golden => self.golden,
        }
    }
}

/// The programmer's bus driver.
///
/// `RX` sizes the receive buffer; program requests are tiny, so it can be
/// small.
pub struct Supervisor<S, W, const RX: usize> {
    store: S,
    writer: W,
    layout: ProgLayout,
    id: u8,
    decoder: Decoder,
    rx: [u8; RX],
    tx: [u8; RESULT_WIRE],
    scratch: [u8; SCRATCH],
}

impl<S: ImageStore, W: NvmWriter, const RX: usize> Supervisor<S, W, RX> {
    /// Creates a supervisor for node `id`.
    pub const fn new(store: S, writer: W, layout: ProgLayout, id: u8) -> Self {
        Self {
            store,
            writer,
            layout,
            id,
            decoder: Decoder::new(),
            rx: [0; RX],
            tx: [0; RESULT_WIRE],
            scratch: [0; SCRATCH],
        }
    }

    /// Programs the target from `source`, returning the outcome.
    pub fn program(&mut self, source: ProgSource) -> ProgStatus {
        let slot = self.layout.slot(source);
        match program(
            &mut self.store,
            &mut self.writer,
            slot.image_offset,
            slot.target_base,
            &mut self.scratch,
        ) {
            Ok(_) => ProgStatus::Ok,
            Err(ProgramError::CorruptSource) => ProgStatus::CorruptSource,
            Err(ProgramError::VerifyFailed) => ProgStatus::VerifyFailed,
            // Store, Nvm, Header, and any future variant.
            Err(_) => ProgStatus::Failed,
        }
    }

    /// Reprograms the main MCU from the golden image. Called by the firmware
    /// when the `TINY_ALIVE` heartbeat is lost.
    pub fn recover(&mut self) -> ProgStatus {
        self.program(ProgSource::Golden)
    }

    /// Feeds one received wire byte from the main MCU link.
    ///
    /// On a complete, valid `ProgProgram` addressed to this node it runs the
    /// program and returns the COBS-encoded `ProgResult` to transmit, otherwise
    /// `None`.
    pub fn feed(&mut self, byte: u8) -> Option<&[u8]> {
        let Ok(Some(len)) = self.decoder.feed(byte, &mut self.rx) else {
            return None;
        };

        let source = {
            let frame = self.rx.get(..len)?;
            let packet = Packet::parse(frame).ok()?;
            if packet.id != self.id || packet.kind != Kind::ProgProgram {
                return None;
            }
            ProgSource::from_code(*packet.payload.first()?)?
        };

        let status = self.program(source);

        let mut raw = [0u8; RESULT_FRAME];
        let raw_len =
            Packet::write(self.id, Kind::ProgResult, &[status.to_code()], &mut raw).ok()?;
        let wire_len = encode_frame(raw.get(..raw_len)?, &mut self.tx)?;
        self.tx.get(..wire_len)
    }
}

#[cfg(test)]
mod tests {
    use cellboot::image::{HEADER_LEN, ImageHeader, ImageKind, Region};
    use cellboot::io::{ImageStore, NvmWriter};
    use cellguard_protocol::{Decoder, Kind, Packet, ProgStatus, encode_frame};

    use super::{ProgLayout, SourceSlot, Supervisor};

    const STORE_CAP: usize = 2048;
    const FLASH_CAP: usize = 1024;
    const NODE: u8 = 3;
    const APP_OFFSET: u32 = 0;
    const GOLDEN_OFFSET: u32 = 1024;

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
            golden: SourceSlot {
                image_offset: GOLDEN_OFFSET,
                target_base: 0,
            },
        }
    }

    fn make() -> Supervisor<MockStore, MockWriter, 64> {
        let store = MockStore {
            buf: [0; STORE_CAP],
        };
        let writer = MockWriter {
            flash: [0; FLASH_CAP],
            finished: false,
        };
        Supervisor::new(store, writer, layout(), NODE)
    }

    #[test]
    fn programs_via_request_packet() {
        let mut sup = make();
        let payload: [u8; 200] = core::array::from_fn(|i| u8::try_from(i % 251).unwrap());
        stage(&mut sup.store, APP_OFFSET, &payload);

        // Build and COBS-encode a ProgProgram(AppStaged) request.
        let mut raw = [0u8; 32];
        let raw_len = Packet::write(NODE, Kind::ProgProgram, &[0], &mut raw).unwrap();
        let mut wire = [0u8; 48];
        let wire_len = encode_frame(&raw[..raw_len], &mut wire).unwrap();

        let mut response = None;
        for &byte in &wire[..wire_len] {
            if let Some(frame) = sup.feed(byte) {
                let mut copy = [0u8; 32];
                copy[..frame.len()].copy_from_slice(frame);
                response = Some((copy, frame.len()));
            }
        }
        let (wire_resp, n) = response.expect("expected a ProgResult");

        // Decode the response and check it is an Ok result.
        let mut scratch = [0u8; 32];
        let mut decoder = Decoder::new();
        let mut done = None;
        for &byte in &wire_resp[..n] {
            if let Some(m) = decoder.feed(byte, &mut scratch).unwrap() {
                done = Some(m);
            }
        }
        let m = done.unwrap();
        let packet = Packet::parse(&scratch[..m]).unwrap();
        assert_eq!(packet.kind, Kind::ProgResult);
        assert_eq!(packet.payload, &[ProgStatus::Ok.to_code()]);
    }

    #[test]
    fn recover_programs_golden() {
        let mut sup = make();
        let payload: [u8; 128] = core::array::from_fn(|i| u8::try_from(i).unwrap());
        stage(&mut sup.store, GOLDEN_OFFSET, &payload);

        assert_eq!(sup.recover(), ProgStatus::Ok);
        assert!(sup.writer.finished);
        assert_eq!(&sup.writer.flash[..128], &payload[..]);
    }

    #[test]
    fn corrupt_golden_reports_failure() {
        let mut sup = make();
        let payload: [u8; 128] = core::array::from_fn(|i| u8::try_from(i).unwrap());
        stage(&mut sup.store, GOLDEN_OFFSET, &payload);
        // Corrupt a staged byte.
        let base = usize::try_from(GOLDEN_OFFSET).unwrap();
        sup.store.buf[base + HEADER_LEN + 5] ^= 0x01;

        assert_eq!(sup.recover(), ProgStatus::CorruptSource);
        assert!(!sup.writer.finished);
    }
}
