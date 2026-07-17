#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! Bootloader firmware for the CellGuard cellcore (AVR128DA64).
//!
//! On every reset the bootloader checks whether a new application image is
//! staged in EEPROM. If so, it self-programs the application flash section and
//! reboots. Otherwise it jumps straight to the installed application.
//!
//! The boot section is 4 KB (FUSE.BOOTSIZE = 8, flash 0x0000-0x0FFF). The
//! application occupies the remaining 124 KB (flash 0x1000-0x1FFFF).
//!
//! Pin map is from the board schematic (`hardware/boards/cellguard-eval`):
//! - SPI0 (PA4 MOSI, PA5 MISO, PA6 SCK) is the EEPROM bus. App U104 chip-select
//!   is `PG6`, Boot U105 chip-select is `PA7` (both active-low).

use avr_device::avr128da64 as pac;
use avrxt_hal::clock::{self, CcpUnlock, HfFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::Port;
use avrxt_hal::nvmctrl::Nvm;
use avrxt_hal::spi::{Prescaler, Spi};
use cat25::{CAT25128, CAT25M01, Cat25};
use cellboot::drivers::{Cat25Store, EepromState, FlashNvmWriter};
use cellboot::image::Region;
use cellboot::io::{BandedStore, StateStore};
use cellcore::update::state::{self, StagedState, UpdateOutcome};
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::spi::RefCellDevice;

use core::cell::RefCell;
use core::panic::PanicInfo;

/// Core clock frequency, from the external 24 MHz oscillator on PA0/EXTCLK.
const BASE_FREQ: u32 = 24_000_000;

/// App staging EEPROM capacity (U104, CAT25M01, 128 KB).
#[allow(dead_code)]
const APP_CAP: u32 = 128 * 1024;
/// Boot staging EEPROM capacity (U105, CAT25128, 16 KB).
#[allow(dead_code)]
const BOOT_CAP: u32 = 16 * 1024;
/// Cellagent app staging capacity (carved from the end of U104).
#[allow(dead_code)]
const CELLAGENT_CAP: u32 = 4 * 1024;

/// On-chip EEPROM slot holding the probe-able agent state.
const STATE_OFFSET: u16 = 0;
const STATE_LEN: u16 = 64;

/// Flash address where the application begins (right after the 4 KB boot
/// section).
const APP_TARGET_BASE: u32 = 0x1000;

/// Maximum self-program attempts before the bootloader gives up, clears the
/// staged image, and falls through to the installed app.
const MAX_PROGRAM_ATTEMPTS: u8 = 3;

/// This firmware's agent version, reported in the probe status.
const AGENT_VERSION: u32 = 1;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    avr_device::interrupt::disable();
    #[expect(clippy::empty_loop)]
    loop {}
}

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    // Run from the external 24 MHz clock on PA0/EXTCLK.
    clock::set_extclk(&cpu, &dp.CLKCTRL, HfFreq::Mhz24);

    // On-chip NVM: back the agent state with an EEPROM slot, and later use the
    // same NVMCTRL for flash self-programming.
    let nvm = Nvm::new(dp.NVMCTRL);
    let mut state_store = EepromState::new(&nvm, &cpu, STATE_OFFSET, STATE_LEN);
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
    let app = Cat25Store::new(Cat25::new(app_dev, CAT25M01, Delay::new(BASE_FREQ)));
    let boot = Cat25Store::new(Cat25::new(boot_dev, CAT25128, Delay::new(BASE_FREQ)));
    let mut store = BandedStore::new(app, boot);

    // If a verified application image is staged, self-program it into flash.
    if state.staged == StagedState::Ready && state.staged_region == Some(Region::ApplicationCode) {
        if state.program_attempts < MAX_PROGRAM_ATTEMPTS {
            let mut writer = FlashNvmWriter::new(&nvm, &cpu);
            let mut scratch = [0u8; 256];
            match cellprog::programmer::program(
                &mut store,
                &mut writer,
                0,
                APP_TARGET_BASE,
                &mut scratch,
            ) {
                Ok(_header) => {
                    state.staged = StagedState::Empty;
                    state.staged_region = None;
                    state.last_outcome = UpdateOutcome::Success;
                    state.program_attempts = 0;
                    let _ = state_store.store(&state.serialize());
                    software_reset(&cpu);
                }
                Err(_) => {
                    state.program_attempts += 1;
                    let _ = state_store.store(&state.serialize());
                    software_reset(&cpu);
                }
            }
        }
        // Attempts exhausted. Clear staged so the device does not loop, and
        // fall through to the installed app. If the error was non-destructive
        // (corrupt source, bad header) the installed app is intact and runs
        // normally. If it was destructive, the cellprog watchdog detects
        // heartbeat loss and attempts recovery.
        state.staged = StagedState::Empty;
        state.staged_region = None;
        state.last_outcome = UpdateOutcome::ProgramFailed;
        state.program_attempts = 0;
        let _ = state_store.store(&state.serialize());
    }

    // No pending update: jump to the installed application.
    unsafe { jump_to_app() }
}

/// Triggers a software reset of the microcontroller.
fn software_reset(cpu: &pac::CPU) -> ! {
    avr_device::interrupt::disable();
    avr_device::interrupt::free(|_| {
        cpu.unlock_ioreg();
        // SAFETY: writing SWRR triggers an immediate microcontroller reset.
        unsafe {
            (*pac::RSTCTRL::ptr())
                .swrr()
                .write(|w| w.swrst().set_bit());
        }
    });
    #[expect(clippy::empty_loop)]
    loop {}
}

/// Jumps to the application at [`APP_TARGET_BASE`].
///
/// # Safety
///
/// Transmutes a flash address to a function pointer. The caller must guarantee
/// that valid application code is present at that address.
unsafe fn jump_to_app() -> ! {
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
