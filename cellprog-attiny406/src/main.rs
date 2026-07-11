#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use avr_device::attiny406 as pac;
use avrxt_hal::clock::{self, ClkPrescaler, TinyBaseFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::Port;
use avrxt_hal::usart::{Frame, Usart};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::StatefulOutputPin;
use updi::Programmer;

use core::panic::PanicInfo;

use self::updi_link::UsartUpdiLink;

mod updi_link;

/// UPDI baud. Conservative default.
const UPDI_BAUD: u32 = 115_200;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Firmware has panicked, so stop all interrupts and halt.
    avr_device::interrupt::disable();
    loop {}
}

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Run the main clock at full speed (prescaler off). The ATtiny406 ships with
    // the OSCCFG fuse at 20 MHz. Derive the delay clock from the same base and
    // prescaler so they cannot drift apart.
    const BASE_FREQ: TinyBaseFreq = TinyBaseFreq::Mhz20;
    const PRESCALER: Option<ClkPrescaler> = None;
    clock::set_main_clock_prescaler(&dp.CPU, &dp.CLKCTRL, PRESCALER);
    let f_cpu = BASE_FREQ.clk_per_hz(PRESCALER);
    let mut delay = Delay::new(f_cpu);

    let porta = Port::new(dp.PORTA).split();
    let portb = Port::new(dp.PORTB).split();

    // Route the programmer USART to the target UPDI line. The mux (U1004) selects
    // channel 1 with A1:A0 = 0b01, so A0 (PA4) high and A1 (PA3) low.
    let _mux_a0 = porta.p4.into_output_high();
    let _mux_a1 = porta.p3.into_output();

    // USART0 pins: TxD (PB2) output idle-high, RxD (PB3) input.
    let _tx = portb.p2.into_output_high();
    let _rx = portb.p3.into_input();

    let usart = Usart::with_frame(dp.USART0, f_cpu, UPDI_BAUD, Frame::EIGHT_E_2);
    let mut programmer = Programmer::new(UsartUpdiLink::new(usart));

    // Bring-up smoke test: try to enter programming mode on the target. With no
    // target attached the reads time out and this returns an error, so the blink
    // rate just reports the outcome. The full supervisor (staged-image source,
    // ProgProgram handling, golden recovery) comes next.
    let period_ms = if programmer.enter().is_ok() { 100 } else { 500 };

    let mut heartbeat = porta.p7.into_output_high();
    loop {
        heartbeat.toggle().ok();
        delay.delay_ms(period_ms);
    }
}
