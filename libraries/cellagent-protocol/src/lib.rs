#![no_std]

use zerocopy::{
    FromBytes, Immutable, IntoBytes, KnownLayout, LittleEndian, TryFromBytes, U16, Unaligned,
};

pub mod cobs; // TODO: pub to silence unused warning

#[derive(FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(C)]
pub struct Packet {
    pub header: PacketHeader,
    pub payload: [u8],
}

#[derive(FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(C)]
pub struct PacketHeader {
    pub id: u8,
    pub raw_kind: u8,
    pub crc: U16<LittleEndian>,
}

#[derive(Clone, Copy, TryFromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(u8)]
#[non_exhaustive]
pub enum Kind {
    ReadDeviceId = 1u8.to_le(),
    ReadSerialNumber = 2u8.to_le(),
    ReadTemperature = 3u8.to_le(),
}

/// ATTiny 3-byte device ID.
#[derive(FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(transparent)]
pub struct DeviceId([u8; 3]);

/// 10-byte serial number of the ATTiny MCU.
#[derive(FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(transparent)]
pub struct SerialNumber([u8; 10]);
