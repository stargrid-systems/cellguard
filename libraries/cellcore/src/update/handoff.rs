//! Handing a staged image off to the `cellprog` programmer.
//!
//! After the agent commits an image
//! ([`session::UpdateAgent::pending_program`]), the core MCU tells the
//! programmer to flash it over the local `UART_PROG` link. The programmer reads
//! the image straight from the shared EEPROM, so only a one-byte [`ProgSource`]
//! selector crosses the link, never the image bytes.
//!
//! [`program_frame`] builds the outbound request; [`parse_result`] reads the
//! programmer's reply. The transport (COBS decode, `Packet::parse`) is the
//! caller's, reusing [`cellguard_protocol`] exactly as the field bus does.
//!
//! [`session::UpdateAgent::pending_program`]: crate::update::session::UpdateAgent::pending_program

use cellboot::image::Region;
use cellguard_protocol::{
    HEADER_LEN, Kind, PAYLOAD_CRC_LEN, Packet, ProgSource, ProgStatus, encode_frame,
};

/// Size of a `ProgProgram` frame before COBS: header, one selector byte, and
/// the payload CRC.
const PROGRAM_FRAME: usize = HEADER_LEN + 1 + PAYLOAD_CRC_LEN;

/// Worst-case COBS-encoded size of a `ProgProgram` frame, including the
/// terminator. Size an outbound buffer with this.
pub const PROGRAM_WIRE: usize = PROGRAM_FRAME + PROGRAM_FRAME.div_ceil(254) + 1;

/// Maps a committed region to the programmer source that flashes it.
///
/// Returns `None` for a region that is not a programmable target (the factory
/// region, or any region added later).
#[must_use]
pub const fn source_for(region: Region) -> Option<ProgSource> {
    match region {
        Region::ApplicationCode => Some(ProgSource::AppStaged),
        Region::Bootloader => Some(ProgSource::BootloaderStaged),
        _ => None,
    }
}

/// Builds the COBS-encoded `ProgProgram` frame that tells programmer node
/// `prog_id` to flash the image staged for `region`, writing it into `out`.
///
/// Returns the encoded length, or `None` if `region` is not a programmable
/// target or `out` is smaller than [`PROGRAM_WIRE`].
#[must_use]
pub fn program_frame(prog_id: u8, region: Region, out: &mut [u8]) -> Option<usize> {
    let source = source_for(region)?;
    let mut raw = [0u8; PROGRAM_FRAME];
    let raw_len = Packet::write(prog_id, Kind::ProgProgram, &[source.to_code()], &mut raw).ok()?;
    encode_frame(raw.get(..raw_len)?, out)
}

/// Reads a programmer reply from an already-parsed packet.
///
/// Returns the reported [`ProgStatus`], or `None` if the packet is not a
/// well-formed `ProgResult`.
#[must_use]
pub fn parse_result(packet: &Packet<'_>) -> Option<ProgStatus> {
    if packet.kind != Kind::ProgResult {
        return None;
    }
    ProgStatus::from_code(*packet.payload.first()?)
}

#[cfg(test)]
mod tests {
    use cellboot::image::Region;
    use cellguard_protocol::{Decoder, Kind, Packet, ProgSource, ProgStatus};

    use super::{PROGRAM_WIRE, parse_result, program_frame, source_for};

    const PROG_ID: u8 = 4;

    #[test]
    fn maps_regions_to_sources() {
        assert_eq!(
            source_for(Region::ApplicationCode),
            Some(ProgSource::AppStaged)
        );
        assert_eq!(
            source_for(Region::Bootloader),
            Some(ProgSource::BootloaderStaged)
        );
        assert_eq!(source_for(Region::Factory), None);
    }

    #[test]
    fn builds_a_decodable_program_request() {
        let mut wire = [0u8; PROGRAM_WIRE];
        let len = program_frame(PROG_ID, Region::ApplicationCode, &mut wire).unwrap();

        let mut scratch = [0u8; PROGRAM_WIRE];
        let mut decoder = Decoder::new();
        let mut done = None;
        for &byte in &wire[..len] {
            if let Some(n) = decoder.feed(byte, &mut scratch).unwrap() {
                done = Some(n);
            }
        }
        let packet = Packet::parse(&scratch[..done.unwrap()]).unwrap();
        assert_eq!(packet.id, PROG_ID);
        assert_eq!(packet.kind, Kind::ProgProgram);
        assert_eq!(packet.payload, &[ProgSource::AppStaged.to_code()]);
    }

    #[test]
    fn factory_region_has_no_frame() {
        let mut wire = [0u8; PROGRAM_WIRE];
        assert_eq!(program_frame(PROG_ID, Region::Factory, &mut wire), None);
    }

    #[test]
    fn reads_a_result_packet() {
        let mut raw = [0u8; 64];
        let n = Packet::write(
            PROG_ID,
            Kind::ProgResult,
            &[ProgStatus::Ok.to_code()],
            &mut raw,
        )
        .unwrap();
        let packet = Packet::parse(&raw[..n]).unwrap();
        assert_eq!(parse_result(&packet), Some(ProgStatus::Ok));
    }

    #[test]
    fn rejects_non_result_packet() {
        let mut raw = [0u8; 64];
        let n = Packet::write(PROG_ID, Kind::ProgProgram, &[0], &mut raw).unwrap();
        let packet = Packet::parse(&raw[..n]).unwrap();
        assert_eq!(parse_result(&packet), None);
    }
}
