#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! CellGuard core firmware for the AVR128DA64.
//!
//! Thin hardware wrapper for the core MCU. It brings up the DA64, runs the
//! crypto self-test, stages received images into the external SPI EEPROMs,
//! and hands the peripherals to the shared `cellcore` update agent through
//! `cellcore-runtime`. All update logic lives in those libraries.
//!
//! Linked at 0x2000, after the 8 KB boot section (FUSE.BOOTSIZE = 16). The
//! clock is the 24 MHz external oscillator Y100 on PA0. The pin map is from
//! the board schematic (`hardware/boards/cellguard-eval`).

use core::cell::RefCell;

use avr_device::avr128da64 as pac;
use avrxt_hal::clock::{self, HfFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::Port;
use avrxt_hal::nvmctrl::Nvm;
use avrxt_hal::rtc::{ClockSource, Prescaler, Rtc};
use avrxt_hal::sigrow::Sigrow;
use avrxt_hal::spi::{Prescaler as SpiPrescaler, Spi};
use avrxt_hal::twi::Twi;
use avrxt_hal::usart::{Builder, Frame, Unset, Usart, UsartInstance};
use cat25::{CAT25M01, CAT25128, Cat25};
use cellboot::drivers::{Cat25Store, EepromState};
use cellboot::factory::{self, FactoryRecord};
use cellboot::io::{BandedStore, ImageStore, NoKeyStore};
use cellboot::{layout, state};
use cellcore::balancing::Balancing;
use cellcore::identity::Identity;
use cellcore::update::command::KEY_LEN;
use cellcore::update::dispatch::Dispatcher;
use cellcore::update::session::{RegionSlot, StagingLayout, UpdateAgent};
use cellcore_runtime::{CoreRuntime, TelemetryHandler};
use cellguard_panic::{clear, read_panic_record};
use embedded_hal::spi::{MODE_0, MODE_1};
use embedded_hal_bus::spi::RefCellDevice;

use self::board::Board;

mod board;

/// Y100 external oscillator on PA0, per the verified netlist.
const F_CPU: HfFreq = HfFreq::Mhz24;

/// Debug UART baud (USART5, bring-up).
const BUS_BAUD: u32 = 1_000_000;
/// Baud on the local links to the PROG programmer (USART3) and the
/// cellagent (USART4).
const PROG_BAUD: u32 = 115_200;

/// This node's address on the field bus. Placeholder until provisioned.
const NODE_ID: u8 = 1;
/// The cellagent's node address on its control link.
const CELLAGENT_ID: u8 = 3;
/// The image `target_id` this device accepts. Placeholder until provisioned.
const TARGET_ID: u16 = 1;
/// The cellagent's image target_id. Placeholder until provisioned.
const CELLAGENT_TARGET_ID: u16 = 2;
/// The cellprog programmer's image target_id. Placeholder until provisioned.
const CELLPROG_TARGET_ID: u16 = 3;
/// This firmware's agent version, reported in the probe status.
const AGENT_VERSION: u32 = 1;

/// Consecutive panic-resets before the handler halts instead of resetting.
const PANIC_THRESHOLD: u8 = 3;

const BUS_RX_TIMEOUT_MS: u32 = 10;
/// USART3 receive timeout in ms. Bounds each programmer-link read while a
/// session waits for its next reply.
const PROG_RX_TIMEOUT_MS: u32 = 5;
/// USART4 receive timeout in ms. Bounds a read while waiting for a
/// forwarded cellagent reply.
const AGENT_RX_TIMEOUT_MS: u32 = 2;

/// I2C1 bus speed.
const SCL_HZ: u32 = 100_000;

/// TWI transaction timeout in ms. Bounds a wedged expander: the heartbeat
/// toggle may miss its cadence but the loop never blocks forever.
const TWI_TIMEOUT_MS: u32 = 20;

cellguard_panic::panic_handler!(
    unsafe { pac::Peripherals::steal() },
    layout::PANIC_OFFSET,
    PANIC_THRESHOLD
);

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    // Y100: external 24 MHz oscillator. Set explicitly so the app never
    // depends on the bootloader's clock state.
    clock::set_extclk(&cpu, &dp.CLKCTRL, F_CPU);

    // USART5 debug UART on PG4/PG5, the field bus for bring-up. Default pins
    // are PG0/PG1. PORTMUX ALT1 routes to PG4/PG5 where the serial adapter
    // sits.
    dp.PORTMUX.usartrouteb().modify(|_, w| w.usart5().alt1());
    let portg = Port::new(dp.PORTG).split();
    let _bus_tx = portg.p4.into_output_high();
    let _bus_rx = portg.p5.into_input();
    let bus = build_usart(
        Usart::builder(dp.USART5, F_CPU.hz())
            .baud(BUS_BAUD)
            .rx_timeout_ms(BUS_RX_TIMEOUT_MS),
    );

    // Verify the crypto primitives on this silicon before trusting any
    // image. A miscompiled hash must never authenticate firmware.
    if cellcore::kat::self_test().is_err() {
        halt();
    }

    // Fleet key from the USERROW, agent state in an EEPROM slot.
    let nvm = Nvm::new(dp.NVMCTRL);
    let mut key = [0u8; KEY_LEN];
    if nvm.read_userrow(0, &mut key).is_err() {
        halt();
    }
    let mut state_store = EepromState::new(&nvm, &cpu, layout::STATE_OFFSET, layout::STATE_LEN);
    let boot_state = state::load(&mut state_store, AGENT_VERSION);

    let porta = Port::new(dp.PORTA).split();
    let portb = Port::new(dp.PORTB).split();
    let portc = Port::new(dp.PORTC).split();
    let porte = Port::new(dp.PORTE).split();

    // SPI0 host bus (PA4 MOSI, PA5 MISO, PA6 SCK), the staging EEPROM bus.
    let _mosi = porta.p4.into_output();
    let _miso = porta.p5.into_input();
    let _sck = porta.p6.into_output();
    let spi = RefCell::new(Spi::new(dp.SPI0, MODE_0, SpiPrescaler::Div16));

    // App and Boot chip-selects (active low, idle high).
    let cs_app = portg.p6.into_output_high();
    let cs_boot = porta.p7.into_output_high();

    let app_dev = RefCellDevice::new_no_delay(&spi, cs_app).unwrap_or_else(|_| halt());
    let boot_dev = RefCellDevice::new_no_delay(&spi, cs_boot).unwrap_or_else(|_| halt());
    let app = Cat25Store::new(Cat25::new(app_dev, CAT25M01, Delay::new(F_CPU.hz())));
    let boot = Cat25Store::new(Cat25::new(boot_dev, CAT25128, Delay::new(F_CPU.hz())));
    let store = BandedStore::new(app, boot);

    // Factory identity (U106, CAT25128, CS PG7). A bad record falls back to
    // the SIGROW serial and the unprovisioned board model.
    let cs_factory = portg.p7.into_output_high();
    let factory_dev = RefCellDevice::new_no_delay(&spi, cs_factory).unwrap_or_else(|_| halt());
    let mut factory = Cat25Store::new(Cat25::new(factory_dev, CAT25128, Delay::new(F_CPU.hz())));
    let mut record = [0u8; factory::RECORD_LEN];
    let factory_record = factory
        .read(0, &mut record)
        .ok()
        .and_then(|()| FactoryRecord::parse(&record).ok());
    let identity = Identity::from_factory_record(
        factory_record,
        Sigrow::new(&dp.SIGROW).serial_number(),
        AGENT_VERSION,
    );

    let staging = StagingLayout {
        application: RegionSlot {
            offset: 0,
            capacity: layout::CELLPROG_OFFSET,
        },
        cellagent: RegionSlot {
            offset: layout::CELLAGENT_OFFSET,
            capacity: layout::CELLAGENT_CAP,
        },
        cellprog: RegionSlot {
            offset: layout::CELLPROG_OFFSET,
            capacity: layout::CELLPROG_CAP,
        },
        bootloader: RegionSlot {
            offset: layout::BOOT_BAND_OFFSET,
            capacity: layout::BOOT_EEPROM_CAP,
        },
    };
    let agent = UpdateAgent::new(
        store,
        staging,
        TARGET_ID,
        CELLAGENT_TARGET_ID,
        CELLPROG_TARGET_ID,
        &mut key,
        NoKeyStore,
        state_store,
        boot_state,
    );
    let mut dispatcher = Dispatcher::<_, _, _, 512>::new(agent, NODE_ID);
    dispatcher.set_panic_record(read_panic_record(&nvm, layout::PANIC_OFFSET));

    // USART3 = link to the PROG programmer on the default PB0/PB1 pins.
    let _prog_tx = portb.p0.into_output_high();
    let _prog_rx = portb.p1.into_input();
    let prog = build_usart(
        Usart::builder(dp.USART3, F_CPU.hz())
            .baud(PROG_BAUD)
            .rx_timeout_ms(PROG_RX_TIMEOUT_MS),
    );

    // USART4 (PE4 TxD / PE5 RxD) = control link to the cellagent.
    let _agent_tx = porte.p4.into_output_high();
    let _agent_rx = porte.p5.into_input();
    let agent_link = build_usart(
        Usart::builder(dp.USART4, F_CPU.hz())
            .baud(PROG_BAUD)
            .rx_timeout_ms(AGENT_RX_TIMEOUT_MS),
    );

    // I2C1 (PB2/PB3): expanders and the temperature sensor. Internal
    // pull-ups hold the bus between transactions.
    let _sda = portb.p2.into_input_pullup();
    let _scl = portb.p3.into_input_pullup();
    let twi = Twi::with_timeout_ms(dp.TWI1, F_CPU.hz(), SCL_HZ, TWI_TIMEOUT_MS);

    // Bleed PWM on PB7: TCD0 WOD routed there by PORTMUX ALT1. The pin is a
    // plain output (idle low) for when the TCD output is disabled.
    dp.PORTMUX.tcdroutea().modify(|_, w| w.tcd0().alt1());
    let _bleed_wo = portb.p7.into_output();

    // Balancing-test hardware. Rail mux PE0-PE2, INA_EN PF5, gate-off PB5,
    // TINY_ALL_OFF readback PC7, cellagent ALIVE PG0, ADS131M08s on SPI1
    // (CS A PC3, CS B PB6, DRDY A/B PE6/PE7, shared SYNC/RESET PF3) and the
    // IR mux PE3/PF4 parked on the current-sense position.
    let portf = Port::new(dp.PORTF).split();
    let mut delay = Delay::new(F_CPU.hz());
    let spi1 = Spi::new(dp.SPI1, MODE_1, SpiPrescaler::Div16);
    let board = Board::new(
        twi,
        &cpu,
        dp.VREF,
        dp.ADC0,
        dp.TCD0,
        &mut delay,
        spi1,
        portf.p3.into_output_high(),
        portc.p3.into_output_high(),
        portb.p6.into_output_high(),
        porte.p3.into_output(),
        portf.p4.into_output(),
        porte.p0.into_output(),
        porte.p1.into_output(),
        porte.p2.into_output(),
        portf.p5.into_output(),
        portb.p5.into_output(),
        portc.p7.into_input(),
        portg.p0.into_input(),
        porte.p6.into_input(),
        porte.p7.into_input(),
    );
    let mut node = NodeTelemetry::new(identity, board);

    let mut runtime = CoreRuntime::new(dispatcher, bus, prog, agent_link, CELLAGENT_ID)
        .with_telemetry(&mut node, NODE_ID);

    // This boot is healthy, so any prior panic was transient. Clear the
    // crash-loop counter.
    clear(&nvm, &cpu, layout::PANIC_OFFSET);

    // RTC as the free-running time base (~1.024 kHz). The heartbeat cadence
    // lives in on_tick.
    let rtc = Rtc::new(dp.RTC, ClockSource::Internal1k, Prescaler::Div1, u16::MAX);

    loop {
        runtime.tick(u32::from(rtc.count()));
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

/// Halts with interrupts disabled.
fn halt() -> ! {
    avr_device::interrupt::disable();
    loop {
        core::hint::spin_loop();
    }
}

/// Node-local request kinds: the device identity, then the balancing layer
/// over this board. The orphan rule wants this wrapper in the firmware
/// crate.
struct NodeTelemetry {
    identity: Identity,
    balancing: Balancing<Board>,
}

impl NodeTelemetry {
    fn new(identity: Identity, board: Board) -> Self {
        Self {
            identity,
            balancing: Balancing::new(board),
        }
    }
}

impl TelemetryHandler for NodeTelemetry {
    fn handle(
        &mut self,
        now: u32,
        kind: cellguard_protocol::Kind,
        payload: &[u8],
        out: &mut [u8],
    ) -> Option<(cellguard_protocol::Kind, usize)> {
        if let Some(reply) = self.identity.handle(kind, out) {
            return Some(reply);
        }
        Balancing::handle(&mut self.balancing, now, kind, payload, out)
    }

    fn note_forwarded(&mut self, kind: cellguard_protocol::Kind, payload: &[u8]) {
        if kind == cellguard_protocol::Kind::SetBalancer
            && let Some(&mask) = payload.first()
        {
            self.balancing.note_gate_mask(mask);
        }
    }

    fn on_tick(&mut self, now: u32) {
        self.balancing.hw_mut().poll_adcs();
        self.balancing.tick(now);
        if let Ok(now) = u16::try_from(now) {
            self.balancing.hw_mut().heartbeat(now);
        }
    }
}
