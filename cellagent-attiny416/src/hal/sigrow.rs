//! Access to the signature row memory area.

use core::ptr;

use crate::pac::SIGROW;
use crate::pac::sigrow::{DEVICEID, SERNUM};

/// Reads the 3-byte device ID.
pub fn read_device_id() -> [u8; 3] {
    // SAFETY: Reading the device ID registers is safe at any time.
    unsafe { ptr::read_volatile(const { deviceid_register_block() }) }
}

/// Reads the 10-byte unique serial number.
pub fn read_serial_number() -> [u8; 10] {
    // SAFETY: Reading the serial number registers is safe at any time.
    unsafe { ptr::read_volatile(const { sernum_register_block() }) }
}

const fn deviceid_register_block() -> *const [u8; 3] {
    // SAFETY: SIGROW is safe to access at any time
    let sigrow = unsafe { &*SIGROW::ptr() };
    let start: &DEVICEID = sigrow.deviceid(0);
    start as *const DEVICEID as *const [u8; 3]
}

const fn sernum_register_block() -> *const [u8; 10] {
    // SAFETY: SIGROW is safe to access at any time
    let sigrow = unsafe { &*SIGROW::ptr() };
    let start: &SERNUM = sigrow.sernum(0);
    start as *const SERNUM as *const [u8; 10]
}
