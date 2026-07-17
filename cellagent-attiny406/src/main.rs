#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! Cellagent firmware for the production CellGuard board (ATtiny406, U403).
//!
//! The cellagent is the balancer and safety co-processor. It drives the active
//! balancer gates (GATE_A, GATE_B) and the emergency gate-off
//! (OUT_TINY_ALL_OFF), reads the LM61 temperature sensor, and toggles an ALIVE
//! heartbeat pin. It talks to the cellcore over USART0 using the
//! `cellguard-protocol`. All protocol logic lives in the `cellagent` library;
//! this crate only maps it onto the chip.
//!
//! Pin map (see `scratch/hardware/cellagent-mcu.md`):
//! - PA3 = GATE_B, PA4 = GATE_A, PA5 = ALIVE.
//! - PA7 = TEMP (LM61, ADC AIN7).
//! - PB2 = USART0 TxD, PB3 = USART0 RxD.
//! - PC1 = OUT_TINY_ALL_OFF.

use avr_device::attiny406 as pac;
use avrxt_hal::adc::{Adc, Prescaler as AdcPrescaler, TinyResolution};
use avrxt_hal::clock::{self, ClkPrescaler, TinyBaseFreq};
use avrxt_hal::gpio::{Output, Port};
use avrxt_hal::nvmctrl::{Nvm, NvmInstance};
use avrxt_hal::rstctrl::RstInstance;
use avrxt_hal::rtc::{ClockSource, Prescaler as RtcPrescaler, Rtc};
use avrxt_hal::usart::{Frame, Usart};
use cellagent::{CellagentRuntime, GateControl, TempSensor};
use embedded_hal::digital::{OutputPin, StatefulOutputPin};
use panic_log::{Decision, PanicRecord, RECORD_LEN, clear, store_and_decide};

use core::panic::PanicInfo;

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

/// LM61 temperature input on PA7 (ADC AIN7).
const TEMP_ADC_CHANNEL: u8 = 7;

/// LM61 transfer function: V_out (mV) = 300 + 10 * T_C.
const LM61_BIAS_MV: u32 = 300;
const LM61_CENTI_PER_MV: u32 = 10;

/// ADC reference: VDD ~= 3.3 V, 10-bit resolution.
const VDD_MV: u32 = 3300;
const ADC_FULLSCALE: u32 = 1024;

/// Gate mask bits (match the `GateControl` contract).
const GATE_A_BIT: u8 = 0x01;
const GATE_B_BIT: u8 = 0x02;
const ALL_OFF_BIT: u8 = 0x04;

/// On-chip EEPROM offset of the panic record. The ATtiny406 EEPROM is unused
/// otherwise, so the record starts at 0.
const PANIC_OFFSET: u16 = 0;
/// Consecutive panic-resets before the handler halts instead of resetting.
const PANIC_THRESHOLD: u8 = 3;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    // Stop interrupts, then record the panic in on-chip EEPROM and either reset
    // (to recover) or halt (once the crash-loop limit is reached).
    avr_device::interrupt::disable();
    let dp = unsafe { pac::Peripherals::steal() };
    let nvm = Nvm::new(dp.NVMCTRL);
    let flags = dp.RSTCTRL.flags().bits();
    match store_and_decide(&nvm, &dp.CPU, PANIC_OFFSET, PANIC_THRESHOLD, flags, info) {
        Decision::Reset => dp.RSTCTRL.software_reset(&dp.CPU),
        Decision::Halt => loop {
            core::hint::spin_loop();
        },
    }
}

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    // Run the main clock at full speed (20 MHz).
    clock::set_main_clock_prescaler(&cpu, &dp.CLKCTRL, PRESCALER);
    let f_cpu = BASE_FREQ.clk_per_hz(PRESCALER);

    let nvm = Nvm::new(dp.NVMCTRL);

    let porta = Port::new(dp.PORTA).split();
    let portb = Port::new(dp.PORTB).split();
    let portc = Port::new(dp.PORTC).split();

    // Balancer gates and the emergency gate-off. All idle low: gates off and
    // all-off inactive.
    let mut gates = BalancerGates {
        gate_a: porta.p4.into_output(),
        gate_b: porta.p3.into_output(),
        all_off: portc.p1.into_output(),
    };

    // ALIVE heartbeat (PA5). Starts high.
    let mut alive = porta.p5.into_output_high();

    // ADC0 for the LM61 on PA7 (AIN7). VDD reference, 10-bit. The HAL ADC
    // driver keeps the reset-default reference (INTREF), so select VDD first.
    dp.ADC0.ctrlc().modify(|_, w| w.refsel().vddref());
    let mut temp = Lm61Temp {
        adc: Adc::new(dp.ADC0, AdcPrescaler::Div64, TinyResolution::Bits10),
    };

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
    let rtc = Rtc::new(dp.RTC, ClockSource::Internal1k, RtcPrescaler::Div1, u16::MAX);

    let mut runtime = CellagentRuntime::new(NODE_ID);

    // Cache the last panic record for the field-bus probe before clearing it.
    runtime.set_panic_record(read_panic_record(&nvm));

    // Init completed: this boot is healthy, so any prior panic was transient.
    clear(&nvm, &cpu, PANIC_OFFSET);

    let mut last_toggle = rtc.count();
    loop {
        if let Ok(byte) = usart.read_byte() {
            let _ = runtime.service(byte, &mut gates, &mut temp, &mut usart);
        }

        let now = rtc.count();
        if now.wrapping_sub(last_toggle) >= HEARTBEAT_TICKS {
            let _ = alive.toggle();
            last_toggle = now;
        }
    }
}

/// Active balancer gate outputs.
struct BalancerGates {
    gate_a: Output,
    gate_b: Output,
    all_off: Output,
}

impl GateControl for BalancerGates {
    fn set_gates(&mut self, mask: u8) {
        if mask & ALL_OFF_BIT != 0 {
            let _ = self.all_off.set_high();
        } else {
            let _ = self.all_off.set_low();
        }
        if mask & GATE_A_BIT != 0 {
            let _ = self.gate_a.set_high();
        } else {
            let _ = self.gate_a.set_low();
        }
        if mask & GATE_B_BIT != 0 {
            let _ = self.gate_b.set_high();
        } else {
            let _ = self.gate_b.set_low();
        }
    }
}

/// LM61 temperature sensor on ADC0.
struct Lm61Temp {
    adc: Adc<pac::ADC0>,
}

impl TempSensor for Lm61Temp {
    fn read_centi_celsius(&mut self) -> i16 {
        let raw = self.adc.read_channel(TEMP_ADC_CHANNEL);
        let v_mv = u32::from(raw) * VDD_MV / ADC_FULLSCALE;
        i16::try_from(v_mv.saturating_sub(LM61_BIAS_MV) * LM61_CENTI_PER_MV).unwrap_or(0)
    }
}

/// Reads the last panic record from EEPROM, if a valid one is stored.
fn read_panic_record<T: NvmInstance>(nvm: &Nvm<T>) -> Option<PanicRecord> {
    let mut buf = [0u8; RECORD_LEN];
    nvm.read_eeprom(PANIC_OFFSET, &mut buf).ok()?;
    PanicRecord::parse(&buf).ok()
}

/// Halts with interrupts disabled.
fn halt() -> ! {
    avr_device::interrupt::disable();
    #[expect(clippy::empty_loop, reason = "nothing left to do after a fatal init error")]
    loop {}
}
