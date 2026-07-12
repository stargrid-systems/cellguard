#![no_std]
#![no_main]

//! CellGuard core firmware for the AVR128DB48 development board.
//!
//! The DB48 is Simon's devkit, not the populated board (that is the DA64). This
//! is the same thin `cellcore` wrapper as the DA64 target, so the update agent
//! can be exercised by hand. It has none of the board peripherals, so staging
//! uses an in-RAM store and the two links are plain UARTs on placeholder pins.
//! The on-chip EEPROM state store and USERROW key read are real on this
//! silicon.

use avr_device::avr128db48 as pac;
use avrxt_hal::clock::{self, HfFreq};
use avrxt_hal::gpio::Port;
use avrxt_hal::nvmctrl::Nvm;
use avrxt_hal::usart::{Builder, Frame, Unset, Usart, UsartInstance};
use cellboot::drivers::EepromState;
use cellboot::io::NoKeyStore;
use cellcore::update::dispatch::Dispatcher;
use cellcore::update::session::{RegionSlot, StagingLayout, UpdateAgent};
use cellcore::update::state;
use cellcore_runtime::{CoreRuntime, RamImageStore};

use core::panic::PanicInfo;

/// Core clock frequency.
const F_CPU: HfFreq = HfFreq::Mhz24;

/// Field-bus baud (USART0).
const BUS_BAUD: u32 = 115_200;
/// Baud on the link that stands in for the PROG programmer (USART1).
const PROG_BAUD: u32 = 115_200;

/// This node's address on the field bus. Placeholder.
const NODE_ID: u8 = 1;
/// The programmer's node address on the local link. Placeholder.
const PROG_ID: u8 = 2;
/// The image `target_id` this device accepts. Placeholder.
const TARGET_ID: u16 = 1;
/// This firmware's agent version, reported in the probe status.
const AGENT_VERSION: u32 = 1;

/// Fleet HMAC key length in the USERROW.
const KEY_LEN: usize = 16;
/// On-chip EEPROM slot holding the probe-able agent state.
const STATE_OFFSET: u16 = 0;
const STATE_LEN: u16 = 64;

/// In-RAM staging capacity.
const STAGE_CAP: usize = 4096;

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

    let nvm = Nvm::new(dp.NVMCTRL);
    let mut key = [0u8; KEY_LEN];
    if nvm.read_userrow(0, &mut key).is_err() {
        halt();
    }
    let mut state_store = EepromState::new(&nvm, &cpu, STATE_OFFSET, STATE_LEN);
    let boot_state = state::load(&mut state_store, AGENT_VERSION);

    let layout = StagingLayout {
        application: RegionSlot {
            offset: 0,
            capacity: 2048,
        },
        bootloader: RegionSlot {
            offset: 2048,
            capacity: 2048,
        },
    };
    let agent = UpdateAgent::new(
        RamImageStore::<STAGE_CAP>::new(),
        layout,
        TARGET_ID,
        &key,
        NoKeyStore,
        state_store,
        boot_state,
    );
    let dispatcher = Dispatcher::<_, _, _, 512>::new(agent, NODE_ID);

    // USART0 = field bus on the default PA0/PA1 pins.
    let porta = Port::new(dp.PORTA).split();
    let _bus_tx = porta.p0.into_output_high();
    let _bus_rx = porta.p1.into_input();
    let bus = build_usart(Usart::builder(dp.USART0, F_CPU.hz()).baud(BUS_BAUD));

    // USART1 = stand-in PROG link on the default PC0/PC1 pins.
    let portc = Port::new(dp.PORTC).split();
    let _prog_tx = portc.p0.into_output_high();
    let _prog_rx = portc.p1.into_input();
    let prog = build_usart(Usart::builder(dp.USART1, F_CPU.hz()).baud(PROG_BAUD));

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
