#![no_std]
#![no_main]

//! CellGuard core firmware for the AVR128DA64, the MCU populated on the board.
//!
//! This is the thin hardware wrapper for the core MCU. It brings up the DA64
//! and hosts the `cellcore` business logic, which is written against abstract
//! I/O traits and lives in its own crate. Wiring the update agent (the bus
//! transport plus the EEPROM, USERROW-key, and NVM stores) is the next step,
//! shared with the AVR128DB48 devkit target.

use avr_device::avr128da64 as pac;
use avrxt_hal::clock::{self, HfFreq};
use avrxt_hal::delay::Delay;
use embedded_hal::delay::DelayNs;

use core::panic::PanicInfo;

/// Core clock frequency.
const F_CPU: HfFreq = HfFreq::Mhz24;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Firmware has panicked, so stop all interrupts and halt.
    avr_device::interrupt::disable();
    loop {}
}

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    clock::set_oschf(&dp.CPU, &dp.CLKCTRL, F_CPU);
    let mut delay = Delay::new(F_CPU.hz());

    loop {
        delay.delay_ms(1000);
    }
}
