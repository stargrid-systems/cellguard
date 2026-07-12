#![no_std]
#![no_main]

//! CellGuard core firmware for the AVR128DA64, the MCU populated on the board.
//!
//! This is the thin hardware wrapper for the core MCU. It brings up the DA64,
//! runs the mandatory crypto self-test, stages received images into the
//! external SPI EEPROMs, and hands the peripherals to the shared `cellcore`
//! update agent through `cellcore-runtime`. All update logic lives in those
//! libraries. This crate only maps them onto this chip.
//!
//! Pin map is from the board schematic (`hardware/boards/cellguard-eval`):
//! - SPI0 (PA4 MOSI, PA5 MISO, PA6 SCK) is the EEPROM bus. Each of the three
//!   staging EEPROMs has its own active-low chip-select GPIO: App `PG6` (U104),
//!   Boot `PA7` (U105), Factory `PG7` (U106).
//! - USART1 (PC4/PC5) is the RS485 field bus (LAST/upstream link).
//! - USART3 (PB0/PB1) is the local link to the ATtiny406 PROG programmer.
//!
//! The agent stages App and Boot images, banded into one address space, so the
//! Factory (golden) chip is left to the PROG programmer's recovery path.

use avr_device::avr128da64 as pac;
use avrxt_hal::clock::{self, HfFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::Port;
use avrxt_hal::nvmctrl::Nvm;
use avrxt_hal::spi::{Prescaler, Spi};
use avrxt_hal::usart::{Builder, Frame, Unset, Usart, UsartInstance};
use cat25::{CAT25128, CAT25M01, Cat25};
use cellboot::drivers::{Cat25Store, EepromState};
use cellboot::io::NoKeyStore;
use cellcore::update::dispatch::Dispatcher;
use cellcore::update::session::{RegionSlot, StagingLayout, UpdateAgent};
use cellcore::update::state;
use cellcore_runtime::{BandedStore, CoreRuntime};
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::spi::RefCellDevice;

use core::cell::RefCell;
use core::panic::PanicInfo;

/// Core clock frequency. The board has an external 24 MHz oscillator on
/// PA0/EXTCLK, but the HAL only drives the internal HF oscillator so far, so we
/// run the internal one at the same rate. Switching to EXTCLK is a HAL follow-up.
const F_CPU: HfFreq = HfFreq::Mhz24;

/// Field-bus baud (USART1, RS485 LAST link).
const BUS_BAUD: u32 = 115_200;
/// Baud on the local link to the PROG programmer (USART3).
const PROG_BAUD: u32 = 115_200;

/// This node's address on the field bus. Placeholder until provisioned.
const NODE_ID: u8 = 1;
/// The programmer's node address on the local link. Placeholder.
const PROG_ID: u8 = 2;
/// The image `target_id` this device accepts. Placeholder until provisioned.
const TARGET_ID: u16 = 1;
/// This firmware's agent version, reported in the probe status.
const AGENT_VERSION: u32 = 1;

/// Fleet HMAC key length in the USERROW.
const KEY_LEN: usize = 16;
/// On-chip EEPROM slot holding the probe-able agent state.
const STATE_OFFSET: u16 = 0;
const STATE_LEN: u16 = 64;

/// App staging EEPROM capacity (U104, CAT25M01, 128 KB).
const APP_CAP: u32 = 128 * 1024;
/// Boot staging EEPROM capacity (U105, CAT25128, 16 KB).
const BOOT_CAP: u32 = 16 * 1024;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    // Firmware has panicked, so stop all interrupts and halt.
    avr_device::interrupt::disable();
    loop {}
}

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    clock::set_oschf(&cpu, &dp.CLKCTRL, F_CPU);

    // Mandatory: verify the crypto primitives on this silicon before trusting
    // any image. A miscompiled hash must never authenticate firmware.
    if cellcore::kat::self_test().is_err() {
        halt();
    }

    // On-chip NVM: read the fleet key from the USERROW and back the agent state
    // with an EEPROM slot.
    let nvm = Nvm::new(dp.NVMCTRL);
    let mut key = [0u8; KEY_LEN];
    if nvm.read_userrow(0, &mut key).is_err() {
        halt();
    }
    let mut state_store = EepromState::new(&nvm, &cpu, STATE_OFFSET, STATE_LEN);
    let boot_state = state::load(&mut state_store, AGENT_VERSION);

    let porta = Port::new(dp.PORTA).split();
    let portb = Port::new(dp.PORTB).split();
    let portc = Port::new(dp.PORTC).split();
    let portg = Port::new(dp.PORTG).split();

    // SPI0 host bus (PA4 MOSI, PA5 MISO, PA6 SCK), the staging EEPROM bus. Pin
    // directions are the application's job.
    let _mosi = porta.p4.into_output();
    let _miso = porta.p5.into_input();
    let _sck = porta.p6.into_output();
    let spi = RefCell::new(Spi::new(dp.SPI0, MODE_0, Prescaler::Div16));

    // App and Boot chip-selects (active low, idle high).
    let cs_app = portg.p6.into_output_high();
    let cs_boot = porta.p7.into_output_high();

    let app_dev = RefCellDevice::new_no_delay(&spi, cs_app).unwrap_or_else(|_| halt());
    let boot_dev = RefCellDevice::new_no_delay(&spi, cs_boot).unwrap_or_else(|_| halt());
    let app = Cat25Store::new(Cat25::new(app_dev, CAT25M01, Delay::new(F_CPU.hz())));
    let boot = Cat25Store::new(Cat25::new(boot_dev, CAT25128, Delay::new(F_CPU.hz())));
    let store = BandedStore::new(app, boot);

    let layout = StagingLayout {
        application: RegionSlot {
            offset: 0,
            capacity: APP_CAP,
        },
        bootloader: RegionSlot {
            offset: APP_CAP,
            capacity: BOOT_CAP,
        },
    };
    let agent = UpdateAgent::new(
        store,
        layout,
        TARGET_ID,
        &key,
        NoKeyStore,
        state_store,
        boot_state,
    );
    let dispatcher = Dispatcher::<_, _, _, 512>::new(agent, NODE_ID);

    // USART1 = RS485 field bus on PC4/PC5, which is PORTMUX ALT1. The transceiver
    // handles direction (no MCU DE pin).
    dp.PORTMUX.usartroutea().modify(|_, w| w.usart1().alt1());
    let _bus_tx = portc.p4.into_output_high();
    let _bus_rx = portc.p5.into_input();
    let bus = build_usart(Usart::builder(dp.USART1, F_CPU.hz()).baud(BUS_BAUD));

    // USART3 = link to the PROG programmer on the default PB0/PB1 pins.
    let _prog_tx = portb.p0.into_output_high();
    let _prog_rx = portb.p1.into_input();
    let prog = build_usart(Usart::builder(dp.USART3, F_CPU.hz()).baud(PROG_BAUD));

    let mut runtime = CoreRuntime::new(dispatcher, bus, prog, PROG_ID);
    runtime.run();
}

/// Finishes a USART builder as 8N1, halting the core if the baud is unattainable.
fn build_usart<T: UsartInstance>(builder: Builder<T, u32, Unset>) -> Usart<T> {
    match builder.frame(Frame::EIGHT_N_1).build() {
        Ok(usart) => usart,
        Err(_) => halt(),
    }
}

/// Halts with interrupts disabled. A future revision can blink a fault code.
fn halt() -> ! {
    avr_device::interrupt::disable();
    loop {}
}
