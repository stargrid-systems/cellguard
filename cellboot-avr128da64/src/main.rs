#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! Bootloader firmware for the CellGuard cellcore (AVR128DA64).
//!
//! On every reset the bootloader checks whether a new application image is
//! staged in EEPROM. If so, it self-programs the application flash section and
//! reboots. Otherwise it jumps straight to the installed application.
//!
//! The boot section is 8 KB (FUSE.BOOTSIZE = 16, flash 0x0000-0x1FFF). The
//! application occupies the remaining 120 KB (flash 0x2000-0x1FFFF).
//!
//! Pin map is from the board schematic (`hardware/boards/cellguard-eval`):
//! - SPI0 (PA4 MOSI, PA5 MISO, PA6 SCK) is the EEPROM bus. App U104 chip-select
//!   is `PG6`, Boot U105 chip-select is `PA7` (both active-low).

use core::cell::RefCell;

use avr_device::avr128da64 as pac;
use avrxt_hal::clock::{self, HfFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::Port;
use avrxt_hal::nvmctrl::Nvm;
use avrxt_hal::rstctrl::RstInstance;
use avrxt_hal::spi::{Prescaler, Spi};
use cat25::{CAT25M01, CAT25128, Cat25};
use cellboot::drivers::{Cat25Store, EepromState, FlashNvmWriter};
use cellboot::image::Region;
use cellboot::io::{BandedStore, StateStore};
use cellcore::update::state::{self, AppHealth, BOOT_HEALTH_THRESHOLD, StagedState};
use cellguard_panic::clear;
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::spi::RefCellDevice;

/// BRING-UP: internal RC at 4 MHz (Y100 not verified). The app re-configures
/// its own clock at boot, so the handoff is safe regardless of what this
/// bootloader picks.
const F_CPU: HfFreq = HfFreq::Mhz4;

/// On-chip EEPROM slot holding the probe-able agent state.
const STATE_OFFSET: u16 = 0;
const STATE_LEN: u16 = 64;
/// On-chip EEPROM offset of the panic record (after the state slot).
const PANIC_OFFSET: u16 = STATE_LEN;
/// Consecutive panic-resets before the handler halts instead of resetting.
const PANIC_THRESHOLD: u8 = 3;

/// Flash address where the application begins (right after the 8 KB boot
/// section).
const APP_TARGET_BASE: u32 = 0x2000;

/// Maximum self-program attempts before the bootloader gives up, clears the
/// staged image, and falls through to the installed app.
const MAX_PROGRAM_ATTEMPTS: u8 = 3;

/// This firmware's agent version, reported in the probe status.
const AGENT_VERSION: u32 = 1;

cellguard_panic::panic_handler!(
    unsafe { pac::Peripherals::steal() },
    PANIC_OFFSET,
    PANIC_THRESHOLD
);

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();
    let cpu = dp.CPU;

    // BRING-UP: internal RC until Y100 is verified. Explicit so the clock
    // state does not depend on the reset path that led here.
    clock::set_oschf(&cpu, &dp.CLKCTRL, F_CPU);

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
    let app = Cat25Store::new(Cat25::new(app_dev, CAT25M01, Delay::new(F_CPU.hz())));
    let boot = Cat25Store::new(Cat25::new(boot_dev, CAT25128, Delay::new(F_CPU.hz())));
    let mut store = BandedStore::new(app, boot);

    // If a verified application image is staged, self-program it into flash.
    if state.staged == StagedState::Ready && state.staged_region == Some(Region::ApplicationCode) {
        // A corrupt staged source or an unparseable header will never succeed
        // by retrying, so they short-circuit straight to giving up. Transient
        // failures (NVM/store/verify/release) consume a retry attempt and reset.
        let give_up = if state.program_attempts < MAX_PROGRAM_ATTEMPTS {
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
                    state.mark_programmed(Region::ApplicationCode);
                    let _ = state_store.store(&state.serialize());
                    // Fresh application code gets a fresh crash-loop counter.
                    clear(&nvm, &cpu, PANIC_OFFSET);
                    dp.RSTCTRL.software_reset(&cpu);
                }
                Err(cellprog::programmer::ProgramError::CorruptSource)
                | Err(cellprog::programmer::ProgramError::Header(_)) => true,
                Err(_) => {
                    state.program_attempts += 1;
                    let _ = state_store.store(&state.serialize());
                    dp.RSTCTRL.software_reset(&cpu);
                }
            }
        } else {
            true
        };
        // Give up: clear staged so the device does not loop, and fall through
        // to the installed app. If the error was non-destructive (corrupt
        // source, bad header) the installed app is intact and runs normally.
        // If it was destructive, the cellprog watchdog detects heartbeat loss
        // and attempts recovery.
        if give_up {
            state.mark_program_failed();
            let _ = state_store.store(&state.serialize());
        }
    }

    // Every boot that hands control to the app counts toward the health
    // check. The bootloader bumps boot_count on each such boot, and flips
    // app_health to Bad once it reaches BOOT_HEALTH_THRESHOLD without the app
    // having confirmed itself. The app clears the counter via the runtime's
    // first successful field-bus exchange, so a normally-running device stays
    // at boot_count == 0.
    state.boot_count = state.boot_count.saturating_add(1);
    if state.boot_count >= u16::from(BOOT_HEALTH_THRESHOLD) && state.app_health != AppHealth::Bad {
        state.app_health = AppHealth::Bad;
    }
    let _ = state_store.store(&state.serialize());

    // No pending update: jump to the installed application.
    unsafe { jump_to_app() }
}

/// Jumps to the application at [`APP_TARGET_BASE`].
///
/// `APP_TARGET_BASE` is the app's reset vector: the CRT linked at the start of
/// the app's FLASH region (0x2000), which re-initializes the stack pointer,
/// clears `.bss`, copies `.data`, and then calls `main`. So a jump here is
/// functionally a warm reset into the app, and the bootloader's stack and
/// state do not leak.
///
/// Interrupts are disabled first as defense-in-depth, even though the
/// bootloader never enables them.
///
/// Note: on AVR Dx the hardware interrupt-vector table lives at 0x0000, inside
/// this boot section. The app therefore cannot register its own ISRs while the
/// bootloader owns the boot section, and the current app is polling-only by
/// design. If ISRs are ever added to the app, the bootloader must forward the
/// vectors.
///
/// # Safety
///
/// Transmutes a flash address to a function pointer. The caller must guarantee
/// that valid application code with a reset vector is present at that address.
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
