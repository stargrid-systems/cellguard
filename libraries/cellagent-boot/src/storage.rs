use zerocopy::{FromBytes, Immutable, IntoBytes, KnownLayout, NativeEndian, U16, Unaligned};

#[derive(FromBytes, Immutable, IntoBytes, KnownLayout, Unaligned)]
#[repr(C)]
pub struct Header {
    pub version: u8,
}

#[derive(FromBytes, Immutable, IntoBytes, KnownLayout)]
#[repr(C)]
pub struct V1 {
    pub update_count: u16,
    pub app_crc: u16,
}

impl V1 {
    pub const VERSION: u8 = 1;
}
