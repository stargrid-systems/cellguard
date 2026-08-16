#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! CellGuard programmer firmware for the ATtiny406 (U1003), the on-board
//! `cellprog` MCU.
//!
//! The programmer is a servant: it executes one transactional session command
//! at a time (see `cellguard_protocol::session`). The cellcore app is the sole
//! orchestrator. It stages images in the shared EEPROM and drives the session
//! that reflashes the cellagent over UPDI (mux channel 3).
//!
//! The programmer's single USART reaches the outside world only through the
//! U1004 analog mux, so while it talks UPDI its UART link to the cellcore is
//! physically disconnected. Each command is therefore one transaction: decode
//! a complete command on channel 0 (8N1), switch the mux and frame to UPDI
//! (8E2), run one operation, switch back, reply. Exactly one command may be
//! in flight. Commands sent while the mux is away are electrically lost.
//!
//! The programmer also watches the cellcore heartbeat (`AVR64_TO_PROG` on PB4,
//! toggled by the cellcore via the U103 GPIO expander). If the heartbeat goes
//! silent the programmer pulses reset (`RESET_AVR64` on PB0) a bounded number
//! of times, then latches given-up and keeps listening. There is no
//! autonomous reflash tier: the cellcore owns its own recovery through the
//! bootloader, and cellcore boot-section updates are bench-only.
//!
//! The firmware is built to fit the 4 KB flash without panic diagnostics:
//! panics abort in place and the watchdog turns a hang back into a reset. No
//! panic records are written.
//!
//! Pin map (verified, see `scratch/hardware/cellprog-mcu.md`):
//! - USART0 PB2/PB3 -> U1004 mux.
//! - PA3/PA4 = U1004 select A1/A0.
//! - PB4 = `AVR64_TO_PROG` heartbeat input (from U103 P12).
//! - PB0 = `RESET_AVR64` (active-low, via U107 NAND + Q100 to cellcore reset).

use avr_device::attiny406 as pac;
use avrxt_hal::clock::{self, ClkPrescaler, TinyBaseFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::{Output, Port};
use avrxt_hal::rtc::{ClockSource, Prescaler, Rtc};
use avrxt_hal::usart::{Frame, Usart};
use avrxt_hal::wdt::{Period, Watchdog};
use cellguard_protocol::Encoder;
use cellprog::SessionHandler;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use updi::TinyProgrammer;

use self::updi_link::UsartUpdiLink;

mod updi_link;

/// Main clock: 20 MHz internal, prescaler off.
const BASE_FREQ: TinyBaseFreq = TinyBaseFreq::Mhz20;
const PRESCALER: Option<ClkPrescaler> = None;

/// Baud on both the UART command link and the UPDI link.
const BAUD: u32 = 115_200;

/// USART receive timeout. Short enough that the heartbeat is sampled often
/// between commands.
const RX_TIMEOUT_MS: u32 = 50;

/// Heartbeat-loss threshold in RTC ticks. The RTC runs at ~1.024 kHz
/// (Internal1k, prescaler /1), so 2048 ticks is roughly 2 s.
const HEARTBEAT_TIMEOUT_TICKS: u16 = 2048;

/// Reset attempts before giving up.
const MAX_RESETS: u8 = 2;

/// Reset pulse width (PB0 held low), in microseconds.
const RESET_PULSE_US: u32 = 1000;

/// Session idle timeout in RTC ticks (~1.024 kHz): roughly 500 ms. An
/// abandoned session (cellcore reset or power blip mid-session) leaves the
/// cellagent in programming mode. This timeout resets it out so a wedged
/// target cannot stay halted forever.
const SESSION_IDLE_TICKS: u16 = 512;

/// Watchdog period. Bounds a hung UPDI operation: a wedged target can stall
/// one command for seconds, so the period must exceed the worst command while
/// still turning a wedged programmer into a reset.
const WDT_PERIOD: Period = Period::Clk8k;

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    // Run the main clock at full speed (20 MHz).
    clock::set_main_clock_prescaler(&cpu, &dp.CLKCTRL, PRESCALER);
    let f_cpu = BASE_FREQ.clk_per_hz(PRESCALER);

    // Bounds hangs: panics abort in place (no handler runs), so the watchdog
    // is what restores service after a fault.
    let mut wdt = Watchdog::start(&cpu, dp.WDT, WDT_PERIOD);

    let porta = Port::new(dp.PORTA).split();
    let portb = Port::new(dp.PORTB).split();

    // USART0 = the shared link. Start it as the 8N1 UART command link on mux
    // channel 0. TxD idle-high, RxD input.
    let _tx = portb.p2.into_output_high();
    let _rx = portb.p3.into_input();
    let mut usart = Usart::builder(dp.USART0, f_cpu)
        .baud(BAUD)
        .frame(Frame::EIGHT_N_1)
        .rx_timeout_ms(RX_TIMEOUT_MS)
        .build()
        .unwrap_or_else(|_| halt());

    // U1004 mux select: A1 = PA3, A0 = PA4. Channel 0 is the cellcore UART.
    let mut mux = MuxSelect {
        a1: porta.p3.into_output(),
        a0: porta.p4.into_output(),
    };
    mux.cellcore_uart();

    // PB4: AVR64_TO_PROG heartbeat (cellcore toggles U103 P12 over I2C).
    // Pull-up gives a defined idle level before the cellcore configures U103.
    let mut heartbeat = portb.p4.into_input_pullup();

    // PB0: RESET_AVR64, active-low. Drives U107 (NAND) -> Q100 (BSS138 N-FET)
    // -> cellcore PF6 reset. Idle high (not resetting).
    let mut reset_n = portb.p0.into_output_high();

    // RTC as a free-running time base (~1.024 kHz, ~64 s before wrap).
    let rtc = Rtc::new(dp.RTC, ClockSource::Internal1k, Prescaler::Div1, u16::MAX);

    let mut delay = Delay::new(f_cpu);

    // .bss storage: a zero initializer keeps the whole handler out of .data
    // (no flash image), and the startup runtime zeroes it instead of an
    // unrolled copy loop in main's prologue.
    static mut HANDLER: SessionHandler = SessionHandler::new();
    let handler = unsafe { &mut *core::ptr::addr_of_mut!(HANDLER) };

    let mut last_level = heartbeat.is_high().unwrap_or(true);
    let mut last_edge = rtc.count();
    // BRING-UP gate: the cellcore heartbeat is disabled (I2C blocking bug, see
    // the bring-up test report), so silence must not be read as core death.
    // Recovery only arms once a heartbeat edge has actually been seen since
    // programmer boot. Drop this gate once the TWI timeout fix re-enables the
    // heartbeat.
    let mut heartbeat_seen = false;
    let mut resets = 0u8;
    // Latched once reset escalation is exhausted, so the dead branch does not
    // re-evaluate the timeout every loop iteration. Cleared by any heartbeat
    // edge, which means the cellcore came back.
    let mut recovery_given_up = false;

    let mut last_command = rtc.count();

    loop {
        wdt.feed();

        // --- UART command link (returns within ~RX_TIMEOUT_MS) ---
        if let Ok(byte) = usart.read_byte()
            && let Some(cmd) = handler.decode(byte)
        {
            last_command = rtc.count();
            // Traffic from the cellcore proves it is alive: reset the
            // heartbeat baseline so a long session cannot read as death.
            last_edge = last_command;
            usart.set_frame(Frame::EIGHT_E_2);
            mux.cellagent_updi();
            let reply = {
                let link = UsartUpdiLink::new(&mut usart);
                let mut prog = TinyProgrammer::new(link);
                handler.execute(cmd, &mut prog)
            };
            usart.set_frame(Frame::EIGHT_N_1);
            mux.cellcore_uart();
            send_reply(&mut usart, reply);
        }

        // --- Session idle timeout ---
        let now = rtc.count();
        if handler.in_session() && now.wrapping_sub(last_command) > SESSION_IDLE_TICKS {
            usart.set_frame(Frame::EIGHT_E_2);
            mux.cellagent_updi();
            {
                let link = UsartUpdiLink::new(&mut usart);
                let mut prog = TinyProgrammer::new(link);
                handler.expire(&mut prog);
            }
            usart.set_frame(Frame::EIGHT_N_1);
            mux.cellcore_uart();
        }

        // --- Heartbeat edge detection ---
        let level = heartbeat.is_high().unwrap_or(last_level);
        if level != last_level {
            last_level = level;
            last_edge = rtc.count();
            heartbeat_seen = true;
            resets = 0;
            recovery_given_up = false;
        }

        // --- Heartbeat lost: bounded reset escalation ---
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
                // Exhausted: keep listening, stop recovering. Latch so this
                // branch does not re-evaluate the timeout every iteration.
                recovery_given_up = true;
            }
        }
    }
}

/// COBS-encodes a raw reply frame and writes it to the UART link, one byte at
/// a time. No wire buffer: the reply borrows the handler's frame buffer.
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

/// Halts with interrupts disabled.
fn halt() -> ! {
    avr_device::interrupt::disable();
    #[expect(
        clippy::empty_loop,
        reason = "nothing left to do after a fatal init error"
    )]
    loop {}
}

// Keep the compile-time check that the servant's session-command buffer
// covers the worst-case command wire size, next to the buffers themselves.
const _: () = assert!(
    cellguard_protocol::MAX_COMMAND_WIRE >= cellprog::session::MAX_COMMAND_FRAME,
    "decoder buffer must cover the worst-case command frame"
);
