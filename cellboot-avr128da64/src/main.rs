#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! Bootloader firmware for the CellGuard cellcore (AVR128DA64).
//!
//! On every reset it checks for a staged application image in EEPROM,
//! self-programs it into flash, then jumps to the installed application.
//!
//! The boot section is 8 KB (FUSE.BOOTSIZE = 16, flash 0x0000-0x1FFF). The
//! link enforces the bound through the `__TEXT_REGION_LENGTH__` defsym in
//! `.cargo/config.toml`. Panics abort immediately to keep `core::fmt` out of
//! the section, so a hang is bounded by the watchdog instead. Pin map is
//! from the board schematic (`hardware/boards/cellguard-eval`).

use core::cell::RefCell;

use avr_device::avr128da64 as pac;
use avrxt_hal::clock::{self, HfFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::Port;
use avrxt_hal::nvmctrl::Nvm;
use avrxt_hal::rstctrl::RstInstance;
use avrxt_hal::spi::{Prescaler, Spi};
use avrxt_hal::wdt::{Period, Watchdog};
use cat25::{CAT25M01, CAT25128, Cat25};
use cellboot::drivers::{Cat25Store, EepromState, FlashNvmWriter};
use cellboot::image::Region;
use cellboot::io::{BandedStore, StateStore};
use cellboot::state::{self, AppHealth, BOOT_HEALTH_THRESHOLD, StagedState};
use cellboot::{layout, programmer};
use cellguard_panic::clear;
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::spi::RefCellDevice;

/// BRING-UP: internal RC at 4 MHz (Y100 unverified). The app sets its own
/// clock, so the handoff is safe whatever this bootloader picks.
const F_CPU: HfFreq = HfFreq::Mhz4;

/// Watchdog period for the boot path. Long enough to self-program a full
/// 120 KB image from the staging EEPROM (roughly 6 s at 250 kbit/s SPI plus
/// flash row writes), short enough to bound a hung boot.
const WDT_PERIOD: Period = Period::Clk8k;

/// Flash address where the application begins (right after the boot section).
const APP_TARGET_BASE: u32 = layout::BOOT_SECTION_SIZE;

/// Maximum self-program attempts before giving up, clearing the staged
/// image, and falling through to the installed app.
const MAX_PROGRAM_ATTEMPTS: u8 = 3;

/// This firmware's agent version, reported in the probe status.
const AGENT_VERSION: u32 = 1;

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    // BRING-UP: internal RC until Y100 is verified, set explicitly whatever
    // the reset path.
    clock::set_oschf(&cpu, &dp.CLKCTRL, F_CPU);

    // Bounds the whole boot path. Panics abort immediately, so the watchdog
    // turns a hung boot back into a reset.
    let mut wdt = Watchdog::start(&cpu, dp.WDT, WDT_PERIOD);

    // On-chip NVM: agent state in an EEPROM slot, flash writes through the
    // same NVMCTRL.
    let nvm = Nvm::new(dp.NVMCTRL);
    let mut state_store = EepromState::new(&nvm, &cpu, layout::STATE_OFFSET, layout::STATE_LEN);
    let mut state = state::load(&mut state_store, AGENT_VERSION);

    let porta = Port::new(dp.PORTA).split();
    let portg = Port::new(dp.PORTG).split();

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
    let mut store = BandedStore::new(app, boot);

    if state.staged == StagedState::Ready && state.staged_region == Some(Region::ApplicationCode) {
        let give_up = if state.program_attempts < MAX_PROGRAM_ATTEMPTS {
            wdt.feed();
            let mut writer = FlashNvmWriter::new(&nvm, &cpu);
            let mut scratch = [0u8; 256];
            match programmer::program(&mut store, &mut writer, 0, APP_TARGET_BASE, &mut scratch) {
                Ok(_header) => {
                    state.mark_programmed(Region::ApplicationCode);
                    let _ = state_store.store(&state.serialize());
                    // Fresh application code gets a fresh crash-loop counter.
                    clear(&nvm, &cpu, layout::PANIC_OFFSET);
                    dp.RSTCTRL.software_reset(&cpu);
                }
                Err(err) if programmer::retryable(&err) => {
                    state.program_attempts += 1;
                    let _ = state_store.store(&state.serialize());
                    dp.RSTCTRL.software_reset(&cpu);
                }
                Err(_) => true,
            }
        } else {
            true
        };
        // Give up: clear staged so the device does not loop, and fall
        // through to the installed app. A destructive failure is caught by
        // the cellprog watchdog.
        if give_up {
            state.mark_program_failed();
            let _ = state_store.store(&state.serialize());
        }
    }

    // Each handoff increments boot_count. Once it reaches
    // BOOT_HEALTH_THRESHOLD without the app confirming itself, app_health
    // flips to Bad. The app clears the counter on its first successful
    // field-bus exchange.
    state.boot_count = state.boot_count.saturating_add(1);
    if state.boot_count >= u16::from(BOOT_HEALTH_THRESHOLD) && state.app_health != AppHealth::Bad {
        state.app_health = AppHealth::Bad;
    }
    let _ = state_store.store(&state.serialize());

    // The app does not expect an armed watchdog, so hand over with it stopped.
    wdt.stop(&cpu);

    unsafe { jump_to_app() }
}

/// Jumps to the application at [`APP_TARGET_BASE`], its reset vector. The
/// CRT there re-initializes the stack pointer, clears `.bss`, copies
/// `.data`, and calls `main`, so the jump is a warm reset into the app.
///
/// On AVR Dx the interrupt-vector table lives at 0x0000, inside the boot
/// section. The app is polling-only and cannot register ISRs. If the app
/// ever adds ISRs, the bootloader must forward the vectors.
///
/// # Safety
///
/// Transmutes a flash address to a function pointer. The caller must
/// guarantee that valid application code with a reset vector is present at
/// that address.
unsafe fn jump_to_app() -> ! {
    avr_device::interrupt::disable();
    type Entry = fn() -> !;
    let entry: Entry = unsafe { core::mem::transmute(APP_TARGET_BASE as usize) };
    entry();
}

/// Halts with interrupts disabled.
fn halt() -> ! {
    avr_device::interrupt::disable();
    #[expect(clippy::empty_loop)]
    loop {}
}
