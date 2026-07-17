//! Drives the update agent from the bus.
//!
//! [`Dispatcher`] owns an [`UpdateAgent`] plus the COBS decode state and its
//! receive buffer. Feed it wire bytes one at a time with [`Dispatcher::feed`];
//! when a complete frame addressed to this node carries a bootloader command,
//! it runs the agent and returns the COBS-encoded response to transmit.
//!
//! Frames that fail to decode, fail their CRCs, are addressed to another node,
//! or are not bootloader commands are ignored (no response). Relaying frames
//! for other nodes down the daisy chain is a separate concern and not handled
//! here.

use cellboot::image::Region;
use cellboot::io::{ImageStore, KeyStore, StateStore};
use cellguard_panic::{PanicRecord, RECORD_LEN};
use cellguard_protocol::{Decoder, HEADER_LEN, PAYLOAD_CRC_LEN, Packet, encode_frame};

use crate::update::command::Command;
use crate::update::session::UpdateAgent;
use crate::update::state::STATE_LEN;

/// Largest response payload: a status reply or a panic record, whichever is
/// bigger.
const MAX_RESPONSE_PAYLOAD: usize = if STATE_LEN > RECORD_LEN {
    STATE_LEN
} else {
    RECORD_LEN
};

/// Largest pre-COBS response frame: header + biggest payload + payload CRC.
const MAX_RESPONSE_FRAME: usize = HEADER_LEN + MAX_RESPONSE_PAYLOAD + PAYLOAD_CRC_LEN;

/// Worst-case COBS-encoded size of the largest response, including the
/// terminator. COBS adds one code byte per 254 data bytes plus the delimiter.
const MAX_RESPONSE_WIRE: usize = MAX_RESPONSE_FRAME + MAX_RESPONSE_FRAME.div_ceil(254) + 1;

/// Bus driver for the update agent.
///
/// `RX` sizes the receive buffer and must be large enough for the biggest
/// incoming frame (a `Begin` header, or a `Data` chunk plus its overhead).
pub struct Dispatcher<'k, S, K, St, const RX: usize> {
    agent: UpdateAgent<'k, S, K, St>,
    id: u8,
    decoder: Decoder,
    rx: [u8; RX],
    tx: [u8; MAX_RESPONSE_WIRE],
}

impl<'k, S: ImageStore, K: KeyStore, St: StateStore, const RX: usize> Dispatcher<'k, S, K, St, RX> {
    /// Creates a dispatcher for node `id` around `agent`.
    pub const fn new(agent: UpdateAgent<'k, S, K, St>, id: u8) -> Self {
        Self {
            agent,
            id,
            decoder: Decoder::new(),
            rx: [0; RX],
            tx: [0; MAX_RESPONSE_WIRE],
        }
    }

    /// Returns the wrapped agent, e.g. to read its status or check
    /// [`UpdateAgent::pending_program`].
    #[must_use]
    pub const fn agent(&self) -> &UpdateAgent<'k, S, K, St> {
        &self.agent
    }

    /// Returns a mutable reference to the wrapped agent, e.g. to call
    /// [`UpdateAgent::confirm_app_healthy`] after a successful exchange.
    #[must_use]
    pub const fn agent_mut(&mut self) -> &mut UpdateAgent<'k, S, K, St> {
        &mut self.agent
    }

    /// Consumes a staged image as it is handed off to the programmer.
    ///
    /// See [`UpdateAgent::take_pending_program`].
    #[must_use]
    pub fn take_pending_program(&mut self) -> Option<Region> {
        self.agent.take_pending_program()
    }

    /// Caches the last panic record so a later `PanicProbe` reports it. Call
    /// this once at boot after reading the slot from EEPROM.
    pub const fn set_panic_record(&mut self, record: Option<PanicRecord>) {
        self.agent.set_panic_record(record);
    }

    /// Feeds one received wire byte.
    ///
    /// Returns `Some(frame)` with the COBS-encoded response to transmit when a
    /// complete, valid, in-scope command was handled, otherwise `None`.
    pub fn feed(&mut self, byte: u8) -> Option<&[u8]> {
        // `None` mid-frame; `Err` is bus noise the decoder already resynced from.
        let Ok(Some(len)) = self.decoder.feed(byte, &mut self.rx) else {
            return None;
        };

        let frame = self.rx.get(..len)?;
        let packet = Packet::parse(frame).ok()?;
        if packet.id != self.id {
            return None;
        }
        let command = Command::from_packet(packet).ok()?;
        let response = self.agent.handle(command);

        let mut raw = [0u8; MAX_RESPONSE_FRAME];
        let raw_len = response.to_packet(self.id, &mut raw).ok()?;
        let wire_len = encode_frame(raw.get(..raw_len)?, &mut self.tx)?;
        self.tx.get(..wire_len)
    }
}

#[cfg(test)]
mod tests {
    use cellboot::image::{HEADER_LEN, ImageHeader, ImageKind, Region};
    use cellboot::io::{ImageStore, NoKeyStore, StateStore};
    use cellguard_protocol::{Decoder, Encoder, Kind, Packet};
    use hmac_sha256::HMAC;

    use super::Dispatcher;
    use crate::update::session::{RegionSlot, StagingLayout, UpdateAgent};
    use crate::update::state::{PersistentState, StagedState};

    const KEY: [u8; 16] = *b"dispatch-tst-key";
    const TARGET: u16 = 0x33;
    const CELLAGENT_TARGET: u16 = 0x34;
    const NODE: u8 = 7;
    const CAP: usize = 4096;

    struct MemStore {
        buf: [u8; CAP],
    }

    impl ImageStore for MemStore {
        type Error = ();

        fn capacity(&self) -> u32 {
            u32::try_from(CAP).unwrap()
        }

        fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()> {
            let start = usize::try_from(offset).map_err(|_| ())?;
            let end = start.checked_add(buf.len()).ok_or(())?;
            buf.copy_from_slice(self.buf.get(start..end).ok_or(())?);
            Ok(())
        }

        fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), ()> {
            let start = usize::try_from(offset).map_err(|_| ())?;
            let end = start.checked_add(data.len()).ok_or(())?;
            self.buf
                .get_mut(start..end)
                .ok_or(())?
                .copy_from_slice(data);
            Ok(())
        }
    }

    /// A state store that drops writes and reports an empty load.
    struct NullStateStore;

    impl StateStore for NullStateStore {
        type Error = ();

        fn load(&mut self, _buf: &mut [u8]) -> Result<(), ()> {
            Err(())
        }

        fn store(&mut self, _data: &[u8]) -> Result<(), ()> {
            Ok(())
        }
    }

    fn make_dispatcher(
        key: &mut [u8; 16],
    ) -> Dispatcher<'_, MemStore, NoKeyStore, NullStateStore, 512> {
        let layout = StagingLayout {
            application: RegionSlot {
                offset: 0,
                capacity: 2048,
            },
            bootloader: RegionSlot {
                offset: 2048,
                capacity: 2048,
            },
            cellagent: RegionSlot {
                offset: 3072,
                capacity: 1024,
            },
        };
        let agent = UpdateAgent::new(
            MemStore { buf: [0; CAP] },
            layout,
            TARGET,
            CELLAGENT_TARGET,
            key,
            NoKeyStore,
            NullStateStore,
            PersistentState::new(1),
        );
        Dispatcher::new(agent, NODE)
    }

    /// COBS-encodes a command packet the way a host would put it on the wire.
    fn wire_command(kind: Kind, payload: &[u8]) -> ([u8; 256], usize) {
        let mut raw = [0u8; 200];
        let raw_len = Packet::write(NODE, kind, payload, &mut raw).unwrap();
        let mut wire = [0u8; 256];
        let mut encoder = Encoder::new(&raw[..raw_len]);
        let mut pos = 0;
        while let Some(byte) = encoder.pull() {
            wire[pos] = byte;
            pos += 1;
        }
        (wire, pos)
    }

    /// Feeds a wire command into the dispatcher and decodes the response
    /// packet.
    fn exchange(
        dispatcher: &mut Dispatcher<'_, MemStore, NoKeyStore, NullStateStore, 512>,
        kind: Kind,
        payload: &[u8],
    ) -> (Kind, [u8; 64], usize) {
        let (wire, len) = wire_command(kind, payload);
        let mut response = None;
        for &byte in &wire[..len] {
            if let Some(frame) = dispatcher.feed(byte) {
                let mut copy = [0u8; 128];
                copy[..frame.len()].copy_from_slice(frame);
                response = Some((copy, frame.len()));
            }
        }
        let (wire_resp, wire_resp_len) = response.expect("expected a response");

        let mut scratch = [0u8; 128];
        let mut decoder = Decoder::new();
        let mut done = None;
        for &byte in &wire_resp[..wire_resp_len] {
            if let Some(n) = decoder.feed(byte, &mut scratch).unwrap() {
                done = Some(n);
            }
        }
        let n = done.expect("response frame did not complete");
        let packet = Packet::parse(&scratch[..n]).unwrap();
        let mut payload_out = [0u8; 64];
        payload_out[..packet.payload.len()].copy_from_slice(packet.payload);
        (packet.kind, payload_out, packet.payload.len())
    }

    fn signed_image(payload: &[u8]) -> [u8; HEADER_LEN] {
        let header = ImageHeader {
            kind: ImageKind::Application,
            region: Region::ApplicationCode,
            target_id: TARGET,
            fw_version: 5,
            payload_len: 0,
            payload_crc32: 0,
            hmac: [0u8; 32],
        };
        let full = crate::update::verify::sign(header, HMAC::new(KEY), payload).unwrap();
        let mut only_header = [0u8; HEADER_LEN];
        only_header.copy_from_slice(&full);
        only_header
    }

    #[test]
    fn probe_returns_status() {
        let mut key = KEY;
        let mut dispatcher = make_dispatcher(&mut key);
        let (kind, _payload, _len) = exchange(&mut dispatcher, Kind::BootProbe, &[]);
        assert_eq!(kind, Kind::BootStatus);
    }

    #[test]
    fn panic_probe_returns_cached_record() {
        let mut key = KEY;
        let mut dispatcher = make_dispatcher(&mut key);
        let record = cellguard_panic::PanicRecord {
            reset_flags: 0x14,
            consecutive_panics: 2,
            file: {
                let mut f = [0u8; cellguard_panic::FILE_CAP];
                f[..3].copy_from_slice(b"lib");
                f
            },
            file_len: 3,
            line: 40,
            col: 1,
        };
        dispatcher.set_panic_record(Some(record));

        let (kind, payload, len) = exchange(&mut dispatcher, Kind::PanicProbe, &[]);
        assert_eq!(kind, Kind::PanicStatus);
        assert_eq!(len, cellguard_panic::RECORD_LEN);
        let bytes: &[u8; cellguard_panic::RECORD_LEN] =
            payload.get(..len).unwrap().try_into().unwrap();
        assert_eq!(cellguard_panic::PanicRecord::parse(bytes).unwrap(), record);
    }

    #[test]
    fn panic_probe_without_record_is_empty() {
        let mut key = KEY;
        let mut dispatcher = make_dispatcher(&mut key);
        let (kind, _payload, len) = exchange(&mut dispatcher, Kind::PanicProbe, &[]);
        assert_eq!(kind, Kind::PanicStatus);
        assert_eq!(len, 0);
    }

    #[test]
    fn ignores_frame_for_other_node() {
        let mut key = KEY;
        let mut dispatcher = make_dispatcher(&mut key);
        // A well-formed probe addressed to a different node id.
        let mut raw = [0u8; 32];
        let raw_len = Packet::write(NODE + 1, Kind::BootProbe, &[], &mut raw).unwrap();
        let mut encoder = Encoder::new(&raw[..raw_len]);
        let mut got = false;
        while let Some(byte) = encoder.pull() {
            if dispatcher.feed(byte).is_some() {
                got = true;
            }
        }
        assert!(!got, "must not answer a frame addressed to another node");
    }

    #[test]
    #[expect(clippy::cast_possible_truncation, reason = "index < 200 fits in a u8")]
    fn full_update_flow() {
        let mut key = KEY;
        let mut dispatcher = make_dispatcher(&mut key);
        let payload: [u8; 200] = core::array::from_fn(|i| i as u8);
        let header = signed_image(&payload);

        let (kind, _p, _l) = exchange(&mut dispatcher, Kind::BootBegin, &header);
        assert_eq!(kind, Kind::BootAck);

        let mut offset = 0usize;
        for chunk in payload.chunks(32) {
            let mut data = [0u8; 64];
            data[..4].copy_from_slice(&u32::try_from(offset).unwrap().to_le_bytes());
            data[4..4 + chunk.len()].copy_from_slice(chunk);
            let (kind, _p, _l) =
                exchange(&mut dispatcher, Kind::BootData, &data[..4 + chunk.len()]);
            assert_eq!(kind, Kind::BootAck);
            offset += chunk.len();
        }

        let (kind, _p, _l) = exchange(&mut dispatcher, Kind::BootCommit, &[]);
        assert_eq!(kind, Kind::BootAck);
        assert_eq!(
            dispatcher.agent().pending_program(),
            Some(Region::ApplicationCode)
        );
        assert_eq!(dispatcher.agent().status().staged, StagedState::Ready);

        // Handing off consumes the staged image so a reboot cannot re-trigger
        // the same flash.
        assert_eq!(
            dispatcher.take_pending_program(),
            Some(Region::ApplicationCode)
        );
        assert_eq!(dispatcher.agent().pending_program(), None);
        assert_eq!(dispatcher.agent().status().staged, StagedState::Empty);
        assert_eq!(dispatcher.agent().status().app_version, 5);
    }
}
