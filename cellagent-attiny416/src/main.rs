#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! Cellagent devkit firmware for the ATtiny416 Xplained Nano.
//!
//! Mirrors `cellagent-attiny406` but targets the ATtiny416 on the Xplained Nano
//! dev board. The production balancer gates, the LM61, and OUT_TINY_ALL_OFF do
//! not exist on the devkit, so this firmware uses mock hardware: gate state is
//! stored but drives no pins, and the temperature is a fixed value. PB5 (the
//! on-board LED) stands in for the ALIVE heartbeat. USART0 (PB2/PB3) carries
//! the `cellguard-protocol` link as on the production board.

use avr_device::attiny416 as pac;
use avrxt_hal::clock::{self, ClkPrescaler, TinyBaseFreq};
use avrxt_hal::gpio::Port;
use avrxt_hal::nvmctrl::Nvm;
use avrxt_hal::rtc::{ClockSource, Prescaler as RtcPrescaler, Rtc};
use avrxt_hal::usart::{Frame, Usart};
use cellagent::{CellagentRuntime, GateControl, TempSensor};
use cellguard_panic::{clear, read_panic_record};
use embedded_hal::digital::StatefulOutputPin;

/// Main clock: 20 MHz internal, prescaler off.
const BASE_FREQ: TinyBaseFreq = TinyBaseFreq::Mhz20;
const PRESCALER: Option<ClkPrescaler> = None;

/// Baud on the cellcore UART link.
const BAUD: u32 = 115_200;

/// This node's address on the cellcore link. Placeholder until provisioned.
const NODE_ID: u8 = 3;

/// USART receive timeout in ms. Short enough that the heartbeat is serviced
/// even when the link is idle.
const RX_TIMEOUT_MS: u32 = 50;

/// Heartbeat toggle interval in RTC ticks (~1.024 kHz). 256 ticks ~= 250 ms.
const HEARTBEAT_TICKS: u16 = 256;

/// Fixed temperature returned by the mock sensor (25.00 C).
const MOCK_TEMP_CENTI: i16 = 2500;

/// On-chip EEPROM offset of the panic record. The ATtiny416 EEPROM is unused
/// otherwise, so the record starts at 0.
const PANIC_OFFSET: u16 = 0;
/// Consecutive panic-resets before the handler halts instead of resetting.
const PANIC_THRESHOLD: u8 = 3;

cellguard_panic::panic_handler!(
    unsafe { pac::Peripherals::steal() },
    PANIC_OFFSET,
    PANIC_THRESHOLD
);

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    // Run the main clock at full speed (20 MHz).
    clock::set_main_clock_prescaler(&cpu, &dp.CLKCTRL, PRESCALER);
    let f_cpu = BASE_FREQ.clk_per_hz(PRESCALER);

    let nvm = Nvm::new(dp.NVMCTRL);

    let portb = Port::new(dp.PORTB).split();

    // PB5 is the on-board LED (active low) on the Xplained Nano. Used as the
    // ALIVE heartbeat.
    let mut alive = portb.p5.into_output_high();

    // USART0 on PB2 (TxD) / PB3 (RxD), 8N1.
    let _tx = portb.p2.into_output_high();
    let _rx = portb.p3.into_input();
    let mut usart = Usart::builder(dp.USART0, f_cpu)
        .baud(BAUD)
        .frame(Frame::EIGHT_N_1)
        .rx_timeout_ms(RX_TIMEOUT_MS)
        .build()
        .unwrap_or_else(|_| halt());

    // RTC as a free-running time base (~1.024 kHz).
    let rtc = Rtc::new(
        dp.RTC,
        ClockSource::Internal1k,
        RtcPrescaler::Div1,
        u16::MAX,
    );

    let mut runtime = CellagentRuntime::new(NODE_ID);
    let mut gates = MockGates { mask: 0 };
    let mut temp = MockTemp;

    // Cache the last panic record for the field-bus probe before clearing it.
    runtime.set_panic_record(read_panic_record(&nvm, PANIC_OFFSET));

    // Init completed: this boot is healthy, so any prior panic was transient.
    clear(&nvm, &cpu, PANIC_OFFSET);

    let mut last_toggle = rtc.count();
    loop {
        if let Ok(byte) = usart.read_byte() {
            runtime.service(byte, &mut gates, &mut temp, &mut usart);
        }

        let now = rtc.count();
        if now.wrapping_sub(last_toggle) >= HEARTBEAT_TICKS {
            let _ = alive.toggle();
            last_toggle = now;
        }
    }
}

/// Mock balancer gates. Stores the last mask but drives no real pins.
struct MockGates {
    mask: u8,
}

impl GateControl for MockGates {
    fn set_gates(&mut self, mask: u8) {
        self.mask = mask;
    }
}

/// Mock temperature sensor. Always returns a fixed value.
struct MockTemp;

impl TempSensor for MockTemp {
    fn read_centi_celsius(&mut self) -> i16 {
        MOCK_TEMP_CENTI
    }
}

/// Halts with interrupts disabled.
fn halt() -> ! {
    avr_device::interrupt::disable();
    #[expect(
        clippy::empty_loop,
        reason = "nothing left to do after a fatal init error"
    )]
    loop {}
}
