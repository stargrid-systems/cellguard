#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use core::panic::PanicInfo;

use avr_device::asm::delay_cycles;
use avr_device::attiny416 as pac;

use crate::pac::Peripherals;

mod hal;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // disable interrupts - firmware has panicked so no ISRs should continue running
    avr_device::interrupt::disable();

    // get the peripherals so we can access serial and the LED.
    //
    // SAFETY: Because main() already has references to the peripherals this is an
    //         unsafe operation - but because no other code can run after the panic
    //         handler was called, we know it is okay.
    let Peripherals { PORTB, .. } = unsafe { Peripherals::steal() };
    loop {
        set_led(&PORTB, true);
        delay_cycles(500);
        set_led(&PORTB, false);
        delay_cycles(1000);
    }
}

#[avr_device::entry]
fn main() -> ! {
    let Peripherals { PORTB, .. } = unsafe { Peripherals::steal() };

    init_portb(&PORTB);

    loop {
        let switch_pressed = read_switch(&PORTB);
        set_led(&PORTB, switch_pressed);
    }
}

fn init_portb(reg: &pac::PORTB) {
    // Set LED as output
    reg.dirset().write(|w| w.pb5().set_bit());
    // Ensure SW is input
    reg.dirclr().write(|w| w.pb4().set_bit());
    // Enable internal pull-up on SW so it reads high when not pressed
    reg.pin4ctrl().write(|w| w.pullupen().set_bit());
}

fn set_led(reg: &pac::PORTB, on: bool) {
    // From the Users-Guide:
    // > The LED can be activated by driving the connected I/O line to GND.
    reg.out().modify(|_r, w| w.pb5().bit(!on));
}

fn read_switch(reg: &pac::PORTB) -> bool {
    // From the Users-Guide:
    // > when a button is pressed it will drive the I/O line to GND.
    reg.input().read().pb4().bit_is_clear()
}
