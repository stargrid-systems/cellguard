#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

use crate::pac::Peripherals;
use avr_device::asm::delay_cycles;
use avr_device::avr128db48 as pac;

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    // disable interrupts - firmware has panicked so no ISRs should continue running
    avr_device::interrupt::disable();

    // get the peripherals so we can access serial and the LED.
    //
    // SAFETY: Because main() already has references to the peripherals this is an unsafe
    // operation - but because no other code can run after the panic handler was called,
    // we know it is okay.
    let p = unsafe { Peripherals::steal() };
    loop {
        set_led(&p.PORTB, true);
        delay_cycles(500);
        set_led(&p.PORTB, false);
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
    // Set PB3 as output
    reg.dirset().write(|w| w.pb3().set_bit());
    // Ensure PB2 is input
    reg.dirclr().write(|w| w.pb2().set_bit());
    // Enable internal pull-up on PB2 so it reads high when not pressed
    reg.pin2ctrl().write(|w| w.pullupen().set_bit());
}

fn set_led(reg: &pac::PORTB, on: bool) {
    // LED is likely wired active-low; drive low to turn on
    reg.out().modify(|_r, w| w.pb3().bit(!on));
}

fn read_switch(reg: &pac::PORTB) -> bool {
    reg.in_().read().pb2().bit_is_clear()
}
