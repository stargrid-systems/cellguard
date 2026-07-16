//! The `cellprog` supervisor: the programmer's bus-facing driver.
//!
//! [`Supervisor`] owns the staged-image [`ImageStore`] and answers the
//! cellcore over the local `UART_PROG` link, which speaks the same
//! [`cellguard_protocol`] as the field bus.
//!
//! The supervisor does **not** own the [`NvmWriter`]. On the hardware the
//! programmer's single USART is shared between the UART command link and the
//! UPDI link through an analog mux, so the firmware owns the USART and lends
//! the writer to the supervisor only for the duration of a flash. The three
//! phases are therefore split into separate methods so the firmware can switch
//! the USART mode and mux channel between them:
//!
//! 1. [`Supervisor::decode`] feeds one wire byte and, on a complete
//!    [`Kind::ProgProgram`] addressed to this node, returns the source to
//!    flash.
//! 2. The firmware switches the link to UPDI, then calls
//!    [`Supervisor::program`] with the borrowed writer to flash and verify.
//! 3. The firmware switches the link back to UART and calls
//!    [`Supervisor::reply`] to emit the [`Kind::ProgResult`].
//!
//! [`Supervisor::service`] does all three in one call for hosts and tests where
//! the writer is always available.
//!
//! The programmer never acts on its own. The cellcore is the sole
//! orchestrator: it stages images into the EEPROM and sends the program
//! request.

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
}

impl ProgLayout {
    const fn slot(&self, source: ProgSource) -> SourceSlot {
        match source {
            ProgSource::AppStaged => self.app,
            ProgSource::BootloaderStaged => self.bootloader,
        }
    }
}

/// The programmer's bus driver.
///
/// `RX` sizes the receive buffer; program requests are tiny, so it can be
/// small. The [`NvmWriter`] is supplied per flash (see [`Supervisor::program`])
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

    /// Returns a shared reference to the staging store.
    #[must_use]
    pub const fn store(&self) -> &S {
        &self.store
    }

    /// Feeds one received wire byte from the cellcore link.
    ///
    /// On a complete, valid `ProgProgram` addressed to this node, returns the
    /// source to flash. Otherwise returns `None` and the decoder keeps
    /// accumulating.
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

    /// Flashes the staged image for `source` into the target through `writer`,
    /// returning the outcome.
    ///
    /// The writer is borrowed rather than owned so the firmware can lend the
    /// shared USART/UPDI link for just this call.
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
            // Store, Nvm, Header, and any future variant.
            Err(_) => ProgStatus::Failed,
        }
    }

    /// Encodes a `ProgResult(status)` reply into the internal transmit buffer
    /// and returns it, ready to send back on the cellcore link.
    #[must_use]
    pub fn reply(&mut self, status: ProgStatus) -> Option<&[u8]> {
        let mut raw = [0u8; RESULT_FRAME];
        let raw_len = Packet::write(self.id, Kind::ProgResult, &[status.to_code()], &mut raw).ok()?;
        let wire_len = encode_frame(raw.get(..raw_len)?, &mut self.tx)?;
        self.tx.get(..wire_len)
    }

    /// Decodes, programs, and replies in one call, for hosts and tests where
    /// the writer is always available. Returns the encoded reply slice when a
    /// complete request was serviced, otherwise `None`.
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

        // Build and COBS-encode a ProgProgram(AppStaged) request.
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
        // Exercises the three split phases as the firmware will call them.
        let mut sup = make();
        let mut writer = MockWriter {
            flash: [0; FLASH_CAP],
            finished: false,
        };
        let payload: [u8; 128] = core::array::from_fn(|i| u8::try_from(i).unwrap());
        stage(&mut sup.store, APP_OFFSET, &payload);

        // Encode a request.
        let mut raw = [0u8; 32];
        let raw_len = Packet::write(NODE, Kind::ProgProgram, &[0], &mut raw).unwrap();
        let mut wire = [0u8; 48];
        let wire_len = encode_frame(&raw[..raw_len], &mut wire).unwrap();

        // Phase 1: decode.
        let mut source = None;
        for &byte in &wire[..wire_len] {
            if let Some(s) = sup.decode(byte) {
                source = Some(s);
            }
        }
        assert_eq!(source, Some(ProgSource::AppStaged));

        // Phase 2: program.
        let status = sup.program(ProgSource::AppStaged, &mut writer);
        assert_eq!(status, ProgStatus::Ok);
        assert!(writer.finished);
        assert_eq!(&writer.flash[..128], &payload[..]);

        // Phase 3: reply.
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
