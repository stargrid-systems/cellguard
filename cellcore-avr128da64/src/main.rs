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
//! - USART5 (PG4/PG5 via PORTMUX ALT1) is the debug UART for bring-up.
//! - USART3 (PB0/PB1) is the local link to the ATtiny406 PROG programmer.
//!
//! BRING-UP STATE: runs on the default 4 MHz internal RC (Y100 not verified),
//! field bus on USART5 debug UART at 9600 baud, heartbeat disabled (I2C
//! blocking bug). See `scratch/bring-up/test-report.md` for details.

use core::cell::RefCell;

use avr_device::avr128da64 as pac;
use avrxt_hal::clock::HfFreq;
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::Port;
use avrxt_hal::nvmctrl::Nvm;
use avrxt_hal::rtc::{ClockSource, Prescaler as RtcPrescaler, Rtc};
use avrxt_hal::spi::{Prescaler, Spi};
use avrxt_hal::twi::Twi;
use avrxt_hal::usart::{Builder, Frame, Unset, Usart, UsartInstance};
use cat25::{Cat25, CAT25128, CAT25M01};
use cellboot::drivers::{Cat25Store, EepromState};
use cellboot::io::NoKeyStore;
use cellcore::update::dispatch::Dispatcher;
use cellcore::update::session::{RegionSlot, StagingLayout, UpdateAgent};
use cellcore::update::state;
use cellcore_runtime::{BandedStore, CoreRuntime};
use cellguard_panic::{clear, read_panic_record};
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::spi::RefCellDevice;
use tca9535::{Address, PinIndex, Tca9535};

/// BRING-UP: default internal RC at 4 MHz (Y100 not verified).
const F_CPU: HfFreq = HfFreq::Mhz4;

/// Debug UART baud (USART5, bring-up).
const BUS_BAUD: u32 = 9_600;
/// Baud on the local link to the PROG programmer (USART3).
const PROG_BAUD: u32 = 115_200;

/// This node's address on the field bus. Placeholder until provisioned.
const NODE_ID: u8 = 1;
/// The programmer's node address on the local link. Placeholder.
const PROG_ID: u8 = 2;
/// The image `target_id` this device accepts. Placeholder until provisioned.
const TARGET_ID: u16 = 1;
/// The cellagent's image target_id. Placeholder until provisioned.
const CELLAGENT_TARGET_ID: u16 = 2;
/// This firmware's agent version, reported in the probe status.
const AGENT_VERSION: u32 = 1;

/// Fleet HMAC key length in the USERROW.
const KEY_LEN: usize = 16;
/// On-chip EEPROM slot holding the probe-able agent state.
const STATE_OFFSET: u16 = 0;
const STATE_LEN: u16 = 64;
/// On-chip EEPROM offset of the panic record (after the state slot).
const PANIC_OFFSET: u16 = STATE_LEN;
/// Consecutive panic-resets before the handler halts instead of resetting.
const PANIC_THRESHOLD: u8 = 3;

/// App staging EEPROM capacity (U104, CAT25M01, 128 KB).
const APP_CAP: u32 = 128 * 1024;
/// Boot staging EEPROM capacity (U105, CAT25128, 16 KB).
const BOOT_CAP: u32 = 16 * 1024;
/// Cellagent app staging capacity (carved from the end of U104).
const CELLAGENT_CAP: u32 = 4 * 1024;

/// Heartbeat toggle interval in RTC ticks (~1.024 kHz). 256 ticks = 250 ms.
const HEARTBEAT_TICKS: u16 = 256;

/// USART1 receive timeout in ms. Short enough that the heartbeat is serviced.
const BUS_RX_TIMEOUT_MS: u32 = 10;

cellguard_panic::panic_handler!(
    unsafe { pac::Peripherals::steal() },
    PANIC_OFFSET,
    PANIC_THRESHOLD
);

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    // BRING-UP: USART5 debug UART on PG4/PG5. Default USART5 pins are PG0/PG1.
    // PORTMUX ALT1 routes to PG4/PG5 where the DFRobot adapter is connected.
    // Initialized first so diagnostic output is available for every later step.
    dp.PORTMUX.usartrouteb().modify(|_, w| w.usart5().alt1());
    let portg = Port::new(dp.PORTG).split();
    let _bus_tx = portg.p4.into_output_high();
    let _bus_rx = portg.p5.into_input();
    let mut bus = build_usart(
        Usart::builder(dp.USART5, F_CPU.hz())
            .baud(BUS_BAUD)
            .rx_timeout_ms(BUS_RX_TIMEOUT_MS),
    );

    bus.write_byte(b'1');

    // Mandatory: verify the crypto primitives on this silicon before trusting
    // any image. A miscompiled hash must never authenticate firmware.
    if cellcore::kat::self_test().is_err() {
        bus.write_byte(b'K');
        halt();
    }
    bus.write_byte(b'2');

    // On-chip NVM: read the fleet key from the USERROW and back the agent state
    // with an EEPROM slot.
    let nvm = Nvm::new(unsafe { pac::Peripherals::steal().NVMCTRL });
    let mut key = [0u8; KEY_LEN];
    if nvm.read_userrow(0, &mut key).is_err() {
        bus.write_byte(b'U');
        halt();
    }
    bus.write_byte(b'3');
    let mut state_store = EepromState::new(&nvm, &cpu, STATE_OFFSET, STATE_LEN);
    let boot_state = state::load(&mut state_store, AGENT_VERSION);

    let porta = Port::new(dp.PORTA).split();
    let portb = Port::new(dp.PORTB).split();

    // SPI0 host bus (PA4 MOSI, PA5 MISO, PA6 SCK), the staging EEPROM bus.
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
    bus.write_byte(b'4');

    let layout = StagingLayout {
        application: RegionSlot {
            offset: 0,
            capacity: APP_CAP - CELLAGENT_CAP,
        },
        cellagent: RegionSlot {
            offset: APP_CAP - CELLAGENT_CAP,
            capacity: CELLAGENT_CAP,
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
        CELLAGENT_TARGET_ID,
        &mut key,
        NoKeyStore,
        state_store,
        boot_state,
    );
    let mut dispatcher = Dispatcher::<_, _, _, 512>::new(agent, NODE_ID);
    dispatcher.set_panic_record(read_panic_record(&nvm, PANIC_OFFSET));
    bus.write_byte(b'5');

    // USART3 = link to the PROG programmer on the default PB0/PB1 pins.
    let _prog_tx = portb.p0.into_output_high();
    let _prog_rx = portb.p1.into_input();
    let prog = build_usart(Usart::builder(dp.USART3, F_CPU.hz()).baud(PROG_BAUD));
    bus.write_byte(b'6');

    // I2C1 (TWI1) internal bus. PB2 SDA, PB3 SCL (default pins, no PORTMUX).
    // Reaches U103 (TCA9535 @0x20). External pull-ups on the board.
    let mut expander = Tca9535::new(Twi::new(dp.TWI1, F_CPU.hz(), 100_000), Address::Lll);

    // Configure U103 P12 (AVR64_TO_PROG) as output. Cache the output register
    // so the loop toggles with a single I2C write (no read-modify-write).
    let mut heartbeat_out: u16 = {
        let cfg = expander
            .read_configuration()
            .map(|c| c.with_output(PinIndex::P12))
            .unwrap_or_else(|_| tca9535::Configuration(0xFFFF).with_output(PinIndex::P12));
        let _ = expander.write_configuration(cfg);
        let out = expander.read_output().map(|o| o.0).unwrap_or(0xFFFF);
        let out = out | PinIndex::P12.mask();
        let _ = expander.write_output(tca9535::Output(out));
        out
    };
    bus.write_byte(b'7');

    // RTC as a free-running time base (~1.024 kHz, ~64 s before wrap).
    let rtc = Rtc::new(
        dp.RTC,
        ClockSource::Internal1k,
        RtcPrescaler::Div1,
        u16::MAX,
    );
    bus.write_byte(b'8');

    let mut runtime = CoreRuntime::new(dispatcher, bus, prog, PROG_ID);

    // Init completed: this boot is healthy, so any prior panic was transient.
    // Clear the crash-loop counter so unrelated panics do not accumulate.
    clear(&nvm, &cpu, PANIC_OFFSET);

    // BRING-UP: heartbeat disabled. The I2C write_output call blocks
    // indefinitely when the TCA9535 is unreachable, starving the UART
    // poll. Re-enable once the TWI driver has a bounded timeout.
    let _ = expander;
    loop {
        runtime.try_service();
    }
}

/// Finishes a USART builder as 8N1, halting the core if the baud is
/// unattainable.
fn build_usart<T: UsartInstance>(builder: Builder<T, u32, Unset>) -> Usart<T> {
    match builder.frame(Frame::EIGHT_N_1).build() {
        Ok(usart) => usart,
        Err(_) => halt(),
    }
}

/// Halts with interrupts disabled. A future revision can blink a fault code.
fn halt() -> ! {
    avr_device::interrupt::disable();
    loop {
        core::hint::spin_loop();
    }
}
