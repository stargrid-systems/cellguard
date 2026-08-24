#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]
#![expect(
    clippy::similar_names,
    reason = "port names mirror the silicon port letters"
)]

//! `CellGuard` programmer firmware for the `ATtiny406` (U1003). The cellcore
//! orchestrates: the programmer executes one session command at a time and
//! reflashes the cellagent over UPDI (mux channel 3). Self-update sessions
//! are rejected until the verification path fits the flash (issue #60).
//!
//! The single USART reaches the outside only through the U1004 analog mux.
//! While it talks UPDI its UART link to the cellcore is physically
//! disconnected, so exactly one command may be in flight and commands sent
//! while the mux is away are electrically lost.
//!
//! It watches the cellcore heartbeat and, if it goes silent, pulses reset a
//! bounded number of times before latching given-up. There is no autonomous
//! reflash tier.
//!
//! Fits the 4 KB flash without panic diagnostics: panics abort in place, the
//! watchdog turns a hang into a reset, and no panic records are written.
//!
//! Pin map (`scratch/hardware/cellprog-mcu.md`): USART0 PB2/PB3 -> U1004 mux,
//! PA3/PA4 = U1004 select A1/A0, PB4 = `AVR64_TO_PROG` (U103 P12),
//! PB0 = `RESET_AVR64` (active-low, via U107 NAND + Q100 to cellcore reset).

use avr_device::attiny406 as pac;
use avrxt_hal::clock::{self, ClkPrescaler, TinyBaseFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::{Output, Port};
use avrxt_hal::rtc::{ClockSource, Prescaler, Rtc};
use avrxt_hal::usart::{Frame, Usart};
use avrxt_hal::wdt::{Period, Watchdog};
use cellguard_protocol::Encoder;
use cellprog::{SelfStaging, SessionHandler};
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use updi::TinyProgrammer;

use self::updi_link::UsartUpdiLink;

mod updi_link;

const BASE_FREQ: TinyBaseFreq = TinyBaseFreq::Mhz20;
const PRESCALER: Option<ClkPrescaler> = None;

/// Baud on both the UART and UPDI links.
const BAUD: u32 = 115_200;

const RX_TIMEOUT_MS: u32 = 50;

const HEARTBEAT_TIMEOUT_TICKS: u16 = 2048;

const MAX_RESETS: u8 = 2;

const RESET_PULSE_US: u32 = 1000;

/// An abandoned session leaves the cellagent in programming mode. This
/// timeout resets it out.
const SESSION_IDLE_TICKS: u16 = 512;

/// Watchdog period. Bounds a hung UPDI operation: a wedged target can stall
/// one command for seconds, so the period must exceed the worst command while
/// still turning a wedged programmer into a reset.
const WDT_PERIOD: Period = Period::Clk8k;

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap_or_else(|| halt());
    let cpu = dp.CPU;

    clock::set_main_clock_prescaler(&cpu, &dp.CLKCTRL, PRESCALER);
    let f_cpu = BASE_FREQ.clk_per_hz(PRESCALER);

    let mut wdt = Watchdog::start(&cpu, dp.WDT, WDT_PERIOD);

    let porta = Port::new(dp.PORTA).split();
    let portb = Port::new(dp.PORTB).split();

    let _tx = portb.p2.into_output_high();
    let _rx = portb.p3.into_input();
    let mut usart = Usart::builder(dp.USART0, f_cpu)
        .baud(BAUD)
        .frame(Frame::EIGHT_N_1)
        .rx_timeout_ms(RX_TIMEOUT_MS)
        .build()
        .unwrap_or_else(|_| halt());

    let mut mux = MuxSelect {
        a1: porta.p3.into_output(),
        a0: porta.p4.into_output(),
    };
    mux.cellcore_uart();

    // Pull-up: defined idle level until the cellcore configures U103.
    let mut heartbeat = portb.p4.into_input_pullup();

    let mut reset_n = portb.p0.into_output_high();

    let rtc = Rtc::new(dp.RTC, ClockSource::Internal1k, Prescaler::Div1, u16::MAX);

    let mut delay = Delay::new(f_cpu);

    // .bss storage keeps the handler out of .data (no flash image) and lets
    // the startup runtime zero it.
    #[expect(
        clippy::items_after_statements,
        reason = "declared next to its only use, after the bring-up it serves"
    )]
    static mut HANDLER: SessionHandler = SessionHandler::new();
    // SAFETY: the firmware is single-threaded and never enables interrupts,
    // and `HANDLER` is never aliased again after this one reference is taken.
    let handler = unsafe { &mut *core::ptr::addr_of_mut!(HANDLER) };

    let mut last_level = heartbeat.is_high().unwrap_or(true);
    let mut last_edge = rtc.count();
    // BRING-UP gate: silence must not read as core death until the
    // heartbeat has been validated on the bench. Recovery arms only after
    // a real heartbeat edge.
    let mut heartbeat_seen = false;
    let mut resets = 0u8;
    let mut recovery_given_up = false;

    let mut last_command = rtc.count();

    loop {
        wdt.feed();

        if let Ok(byte) = usart.read_byte()
            && let Some(cmd) = handler.decode(byte)
        {
            last_command = rtc.count();
            // Command traffic proves the cellcore is alive: refresh the
            // heartbeat baseline.
            last_edge = last_command;
            if handler.uses_updi(&cmd) {
                link_to_updi(&mut usart, &mut mux);
            }
            let reply = {
                let link = UsartUpdiLink::new(&mut usart);
                let mut prog = TinyProgrammer::new(link);
                handler.execute(cmd, &mut prog, &mut NoStaging)
            };
            link_to_uart(&mut usart, &mut mux);
            send_reply(&mut usart, reply);
        }

        let now = rtc.count();
        if handler.in_session() && now.wrapping_sub(last_command) > SESSION_IDLE_TICKS {
            link_to_updi(&mut usart, &mut mux);
            {
                let link = UsartUpdiLink::new(&mut usart);
                let mut prog = TinyProgrammer::new(link);
                handler.expire(&mut prog);
            }
            link_to_uart(&mut usart, &mut mux);
        }

        let level = heartbeat.is_high().unwrap_or(last_level);
        if level != last_level {
            last_level = level;
            last_edge = rtc.count();
            heartbeat_seen = true;
            resets = 0;
            recovery_given_up = false;
        }

        if heartbeat_seen
            && !recovery_given_up
            && rtc.count().wrapping_sub(last_edge) > HEARTBEAT_TIMEOUT_TICKS
        {
            if resets < MAX_RESETS {
                let _ = reset_n.set_low();
                delay.delay_us(RESET_PULSE_US);
                let _ = reset_n.set_high();
                resets += 1;
                last_edge = rtc.count();
            } else {
                recovery_given_up = true;
            }
        }
    }
}

/// Out-of-line so both switch sites share one body.
#[inline(never)]
fn link_to_updi<T: avrxt_hal::usart::UsartInstance>(usart: &mut Usart<T>, mux: &mut MuxSelect) {
    usart.set_frame(Frame::EIGHT_E_2);
    mux.cellagent_updi();
}

/// Out-of-line so both switch sites share one body.
#[inline(never)]
fn link_to_uart<T: avrxt_hal::usart::UsartInstance>(usart: &mut Usart<T>, mux: &mut MuxSelect) {
    usart.set_frame(Frame::EIGHT_N_1);
    mux.cellcore_uart();
}

/// No wire buffer: streams the encoded reply from the handler's frame buffer.
fn send_reply<T: avrxt_hal::usart::UsartInstance>(usart: &mut Usart<T>, raw: &[u8]) {
    let mut encoder = Encoder::new(raw);
    while let Some(byte) = encoder.pull() {
        usart.write_byte(byte);
    }
}

/// U1004 channel selection on PA3 (A1) and PA4 (A0).
struct MuxSelect {
    a1: Output,
    a0: Output,
}

impl MuxSelect {
    /// Channel 0: cellcore UART (8N1).
    fn cellcore_uart(&mut self) {
        let _ = self.a1.set_low();
        let _ = self.a0.set_low();
    }
    /// Channel 3: cellagent UPDI (8E2).
    fn cellagent_updi(&mut self) {
        let _ = self.a1.set_high();
        let _ = self.a0.set_high();
    }
}

fn halt() -> ! {
    avr_device::interrupt::disable();
    #[expect(
        clippy::empty_loop,
        reason = "nothing left to do after a fatal init error"
    )]
    loop {}
}

/// Self-update staging stub: `End` of a self session always fails its
/// verification check, so nothing ever arms (issue #60).
struct NoStaging;

impl SelfStaging for NoStaging {
    fn read_staged(&mut self, _offset: u16, _buf: &mut [u8]) -> bool {
        false
    }
}

const _: () = assert!(
    cellguard_protocol::MAX_COMMAND_WIRE >= cellprog::MAX_COMMAND_FRAME,
    "decoder buffer must cover the worst-case command frame"
);
