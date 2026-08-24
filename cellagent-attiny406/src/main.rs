#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! Cellagent firmware for the production CellGuard board (ATtiny406, U403).
//! Balancer and safety co-processor. Protocol logic lives in the `cellagent`
//! library.
//!
//! Fits the 4 KB flash without panic diagnostics: panics abort in place, the
//! watchdog turns a hang into a reset, and no panic records are written.
//!
//! Pin map (`scratch/hardware/cellagent-mcu.md`): PA3 = GATE_B, PA4 = GATE_A,
//! PA5 = ALIVE, PA7 = TEMP (LM61, ADC AIN7), PB2/PB3 = USART0 TxD/RxD,
//! PC1 = OUT_TINY_ALL_OFF.

use avr_device::attiny406 as pac;
use avrxt_hal::adc::{Adc, Prescaler as AdcPrescaler, TinyResolution};
use avrxt_hal::clock::{self, ClkPrescaler, TinyBaseFreq};
use avrxt_hal::gpio::{Output, Port};
use avrxt_hal::rtc::{ClockSource, Prescaler as RtcPrescaler, Rtc};
use avrxt_hal::usart::{Frame, Usart};
use avrxt_hal::wdt::{Period, Watchdog};
use cellagent::{CellagentRuntime, GateControl, TempSensor};
use cellguard_protocol::TEMP_INVALID;
use embedded_hal::digital::{OutputPin, StatefulOutputPin};

const BASE_FREQ: TinyBaseFreq = TinyBaseFreq::Mhz20;
const PRESCALER: Option<ClkPrescaler> = None;

const BAUD: u32 = 115_200;

/// Placeholder until provisioned.
const NODE_ID: u8 = 3;

const RX_TIMEOUT_MS: u32 = 50;

const HEARTBEAT_TICKS: u16 = 256;

/// LM61 temperature input on PA7 (ADC AIN7).
const TEMP_ADC_CHANNEL: u8 = 7;

/// LM61 transfer function: V_out (mV) = 300 + 10 * T_C.
const LM61_BIAS_MV: i32 = 300;
const LM61_CENTI_PER_MV: i32 = 10;

/// ADC reference: VDD ~= 3.3 V.
const VDD_MV: i32 = 3300;
const ADC_FULLSCALE: i32 = 1024;

/// Gate mask bits (match the `GateControl` contract).
const GATE_A_BIT: u8 = 0x01;
const GATE_B_BIT: u8 = 0x02;
const ALL_OFF_BIT: u8 = 0x04;

/// Watchdog period. The main loop must always return within one period, even
/// during a response burst, so the USART read timeout bounds one iteration.
const WDT_PERIOD: Period = Period::Clk2k;

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    clock::set_main_clock_prescaler(&cpu, &dp.CLKCTRL, PRESCALER);
    let f_cpu = BASE_FREQ.clk_per_hz(PRESCALER);

    let mut wdt = Watchdog::start(&cpu, dp.WDT, WDT_PERIOD);

    let porta = Port::new(dp.PORTA).split();
    let portb = Port::new(dp.PORTB).split();
    let portc = Port::new(dp.PORTC).split();

    // All three outputs idle low: gates off, all-off inactive.
    let mut gates = BalancerGates {
        gate_a: porta.p4.into_output(),
        gate_b: porta.p3.into_output(),
        all_off: portc.p1.into_output(),
    };

    let mut alive = porta.p5.into_output_high();

    // The HAL ADC driver defaults to INTREF, so select VDD here.
    dp.ADC0.ctrlc().modify(|_, w| w.refsel().vddref());
    let mut temp = Lm61Temp {
        adc: Adc::new(dp.ADC0, AdcPrescaler::Div64, TinyResolution::Bits10),
    };

    let _tx = portb.p2.into_output_high();
    let _rx = portb.p3.into_input();
    let mut usart = Usart::builder(dp.USART0, f_cpu)
        .baud(BAUD)
        .frame(Frame::EIGHT_N_1)
        .rx_timeout_ms(RX_TIMEOUT_MS)
        .build()
        .unwrap_or_else(|_| halt());

    let rtc = Rtc::new(
        dp.RTC,
        ClockSource::Internal1k,
        RtcPrescaler::Div1,
        u16::MAX,
    );

    let mut runtime = CellagentRuntime::new(NODE_ID);

    gates.set_gates(cellagent::SAFE_GATE_MASK);

    let mut last_toggle = rtc.count();
    loop {
        wdt.feed();

        let now = rtc.count();
        if let Ok(byte) = usart.read_byte() {
            runtime.service(byte, now, &mut gates, &mut temp, &mut usart);
        }

        runtime.check_timeout(now, &mut gates);

        if now.wrapping_sub(last_toggle) >= HEARTBEAT_TICKS {
            let _ = alive.toggle();
            last_toggle = now;
        }
    }
}

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

struct Lm61Temp {
    adc: Adc<pac::ADC0>,
}

impl TempSensor for Lm61Temp {
    fn read_centi_celsius(&mut self) -> i16 {
        let raw = self.adc.read_channel(TEMP_ADC_CHANNEL);
        let v_mv = i32::from(raw) * VDD_MV / ADC_FULLSCALE;
        i16::try_from((v_mv - LM61_BIAS_MV) * LM61_CENTI_PER_MV).unwrap_or(TEMP_INVALID)
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
