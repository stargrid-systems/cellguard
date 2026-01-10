#![no_std]

mod storage;

pub struct CellagentBoot {
    _private: (),
}

pub fn run() -> ! {
    // TODO: check if we want to stay in bootloader
    // TODO: validate application crc (if it doesn't match, stay in bootloader)

    todo!()
}
