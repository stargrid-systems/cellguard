#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::panic::PanicInfo;

use avr_device::attiny416 as pac;
use avrxt_hal::clock::{self, ClkPrescaler, TinyBaseFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::Port;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::StatefulOutputPin;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Firmware has panicked, so stop all interrupts and halt.
    avr_device::interrupt::disable();
    loop {}
}

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Run the main clock at full speed (prescaler off). The ATtiny416 Xplained
    // Nano ships with the OSCCFG fuse at 20 MHz. Derive the delay clock from the
    // same base and prescaler so they cannot drift apart.
    const BASE_FREQ: TinyBaseFreq = TinyBaseFreq::Mhz20;
    const PRESCALER: Option<ClkPrescaler> = None;
    clock::set_main_clock_prescaler(&dp.CPU, &dp.CLKCTRL, PRESCALER);
    let mut delay = Delay::new(BASE_FREQ.clk_per_hz(PRESCALER));

    // PB5 drives the on-board LED (active low). Start high, i.e. off.
    let pins = Port::new(dp.PORTB).split();
    let mut led = pins.p5.into_output_high();

    loop {
        led.toggle().ok();
        delay.delay_ms(250);
    }
}
