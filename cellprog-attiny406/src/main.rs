#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! CellGuard programmer firmware for the ATtiny406 (U1003), the on-board
//! `cellprog` MCU.
//!
//! This MCU reflashes the cellcore (AVR128) over UPDI, reading staged images
//! from the shared SPI EEPROM. The cellcore is the sole orchestrator for field
//! updates: it stages images, then sends a `ProgProgram` packet over its
//! USART3, which reaches this MCU's USART0 through the U1004 analog mux
//! (channel 0).
//!
//! The programmer also watches the cellcore heartbeat (`AVR64_TO_PROG` on PB4,
//! toggled by the cellcore via the U103 GPIO expander). If the heartbeat goes
//! silent the programmer recovers the cellcore autonomously: first a reset
//! (PB0 / `RESET_AVR64`), then, if that fails, a reflash of the staged
//! application over UPDI. After a few failed attempts it gives up and keeps
//! listening. There is no automatic recovery for the cellagent.
//!
//! USART0 is shared between the UART command link (mux channel 0, 8N1) and the
//! UPDI link (mux channel 1, 8E2). The firmware listens on channel 0, and on a
//! `ProgProgram` (or a recovery reflash) it switches USART0 to 8E2 and the mux
//! to channel 1, flashes the staged image, then switches back.
//!
//! Pin map (verified, see `scratch/hardware/cellprog-mcu.md`):
//! - USART0 PB2/PB3 -> U1004 mux.
//! - PA3/PA4 = U1004 select A1/A0.
//! - PB4 = `AVR64_TO_PROG` heartbeat input (from U103 P12).
//! - PB0 = `RESET_AVR64` (active-low, via U107 NAND + Q100 to cellcore reset).
//! - SPI0_ALT (PC0 SCK, PC1 MISO, PC2 MOSI). App EEPROM U104 CS = PA2, Boot
//!   EEPROM U105 CS = PC3.

use core::cell::RefCell;

use avr_device::attiny406 as pac;
use avrxt_hal::clock::{self, ClkPrescaler, TinyBaseFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::{Output, Port};
use avrxt_hal::nvmctrl::Nvm;
use avrxt_hal::rtc::{ClockSource, Prescaler, Rtc};
use avrxt_hal::spi::{Prescaler as SpiPrescaler, Spi};
use avrxt_hal::usart::{Frame, Usart};
use cat25::{CAT25M01, CAT25128, Cat25};
use cellboot::drivers::Cat25Store;
use cellboot::io::BandedStore;
use cellguard_panic::clear;
use cellguard_protocol::ProgSource;
use cellprog::supervisor::{ProgLayout, SourceSlot, Supervisor};
use cellprog::writer::UpdiNvmWriter;
use embedded_hal::delay::DelayNs;
use embedded_hal::digital::{InputPin, OutputPin};
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::spi::RefCellDevice;
use embedded_io::Write;
use updi::{Programmer, TinyProgrammer};

use self::updi_link::UsartUpdiLink;

mod updi_link;

/// Main clock: 20 MHz internal, prescaler off.
const BASE_FREQ: TinyBaseFreq = TinyBaseFreq::Mhz20;
const PRESCALER: Option<ClkPrescaler> = None;

/// Baud on both the UART command link and the UPDI link.
const BAUD: u32 = 115_200;

/// This node's address on the cellcore link.
const NODE_ID: u8 = 2;

/// App staging EEPROM capacity (U104, CAT25M01, 128 KB).
const APP_CAP: u32 = 128 * 1024;
/// Cellagent app staging capacity (carved from the end of U104).
const CELLAGENT_CAP: u32 = 4 * 1024;

/// Boot section size on the AVR128DA64 (FUSE.BOOTSIZE = 16, units of 512
/// bytes).
///
/// The bootloader self-programs the application from EEPROM at boot. Cellprog
/// flashes the bootloader itself over UPDI (rare) and serves as fallback for
/// catastrophic recovery via the heartbeat watchdog.
const BOOT_SIZE: u32 = 16 * 512;

/// Boot section starts at flash address 0 on AVR Dx.
const BOOT_TARGET_BASE: u32 = 0x0000;

/// Application starts right after the boot section.
const APP_TARGET_BASE: u32 = BOOT_SIZE;

/// tinyAVR flash offset 0. The data-space base (0x8000) is added internally
/// by TinyProgrammer.
const CELLAGENT_TARGET_BASE: u32 = 0x0000;

/// USART receive timeout. Short enough that the heartbeat is sampled often.
const RX_TIMEOUT_MS: u32 = 50;

/// Heartbeat-loss threshold in RTC ticks. The RTC runs at ~1.024 kHz
/// (Internal1k, prescaler /1), so 2048 ticks is roughly 2 s.
const HEARTBEAT_TIMEOUT_TICKS: u16 = 2048;

/// Reset attempts before escalating to a reflash.
const MAX_RESETS: u8 = 2;

/// Reflash attempts before giving up.
const MAX_REFLASHES: u8 = 2;

/// Reset pulse width (PB0 held low), in microseconds.
const RESET_PULSE_US: u32 = 1000;

/// On-chip EEPROM offset of the panic record. The ATtiny406 EEPROM is unused
/// otherwise, so the record starts at 0.
const PANIC_OFFSET: u16 = 0;
/// Consecutive panic-resets before the handler halts instead of resetting.
const PANIC_THRESHOLD: u8 = 3;

cellguard_panic::panic_handler!(
    unsafe { pac::Peripherals::steal() },
    PANIC_OFFSET,
    PANIC_THRESHOLD
);

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

    // SPI0 on its alternate (PORTC) pins: PC0 SCK, PC1 MISO, PC2 MOSI.
    dp.PORTMUX.ctrlb().write(|w| w.spi0().set_bit());
    let _sck = portc.p0.into_output();
    let _miso = portc.p1.into_input();
    let _mosi = portc.p2.into_output();
    let spi = RefCell::new(Spi::new(dp.SPI0, MODE_0, SpiPrescaler::Div16));

    // App and Boot chip-selects (active low, idle high). PC3 also doubles as
    // the SPI hardware SS. CTRLB.SSD keeps it usable as a GPIO CS.
    let cs_app = porta.p2.into_output_high();
    let cs_boot = portc.p3.into_output_high();
    let app_dev = RefCellDevice::new_no_delay(&spi, cs_app).unwrap_or_else(|_| halt());
    let boot_dev = RefCellDevice::new_no_delay(&spi, cs_boot).unwrap_or_else(|_| halt());
    let app = Cat25Store::new(Cat25::new(app_dev, CAT25M01, Delay::new(f_cpu)));
    let boot = Cat25Store::new(Cat25::new(boot_dev, CAT25128, Delay::new(f_cpu)));
    let store = BandedStore::new(app, boot);

    // The boot image sits at the App/Boot band boundary in the store (the boot
    // EEPROM rebased to offset APP_CAP).
    let layout = ProgLayout {
        app: SourceSlot {
            image_offset: 0,
            target_base: APP_TARGET_BASE,
        },
        bootloader: SourceSlot {
            image_offset: APP_CAP,
            target_base: BOOT_TARGET_BASE,
        },
        cellagent: SourceSlot {
            image_offset: APP_CAP - CELLAGENT_CAP,
            target_base: CELLAGENT_TARGET_BASE,
        },
    };
    let mut supervisor = Supervisor::<_, 128>::new(store, layout, NODE_ID);

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

    let mut last_level = heartbeat.is_high().unwrap_or(true);
    let mut last_edge = rtc.count();
    let mut resets = 0u8;
    let mut reflashes = 0u8;
    // Latched once both recovery tiers are exhausted, so the dead branch does
    // not re-evaluate the timeout every loop iteration. Cleared by any
    // heartbeat edge, which means the cellcore came back.
    let mut recovery_given_up = false;

    // Init completed: this boot is healthy, so any prior panic was transient.
    clear(&nvm, &cpu, PANIC_OFFSET);

    loop {
        // --- UART command link (returns within ~RX_TIMEOUT_MS) ---
        if let Ok(byte) = usart.read_byte()
            && let Some(source) = supervisor.decode(byte)
        {
            usart.set_frame(Frame::EIGHT_E_2);
            let status = match source {
                ProgSource::CellagentAppStaged => {
                    mux.cellagent_updi();
                    let link = UsartUpdiLink::new(&mut usart);
                    let mut writer = UpdiNvmWriter::new(TinyProgrammer::new(link));
                    supervisor.program(source, &mut writer)
                }
                _ => {
                    mux.cellcore_updi();
                    let link = UsartUpdiLink::new(&mut usart);
                    let mut writer = UpdiNvmWriter::new(Programmer::new(link));
                    supervisor.program(source, &mut writer)
                }
            };
            usart.set_frame(Frame::EIGHT_N_1);
            mux.cellcore_uart();
            if let Some(reply) = supervisor.reply(status) {
                let _ = usart.write_all(reply);
            }
        }

        // --- Heartbeat edge detection ---
        let level = heartbeat.is_high().unwrap_or(last_level);
        if level != last_level {
            last_level = level;
            last_edge = rtc.count();
            resets = 0;
            reflashes = 0;
            recovery_given_up = false;
        }

        // --- Heartbeat lost: tiered recovery ---
        if !recovery_given_up && rtc.count().wrapping_sub(last_edge) > HEARTBEAT_TIMEOUT_TICKS {
            if resets < MAX_RESETS {
                // Tier 1: pulse RESET_AVR64 low.
                let _ = reset_n.set_low();
                delay.delay_us(RESET_PULSE_US);
                let _ = reset_n.set_high();
                resets += 1;
                last_edge = rtc.count();
            } else if reflashes < MAX_REFLASHES {
                // Tier 2: reflash the staged application.
                usart.set_frame(Frame::EIGHT_E_2);
                mux.cellcore_updi();
                let _ = {
                    let link = UsartUpdiLink::new(&mut usart);
                    let mut writer = UpdiNvmWriter::new(Programmer::new(link));
                    supervisor.program(ProgSource::AppStaged, &mut writer)
                };
                usart.set_frame(Frame::EIGHT_N_1);
                mux.cellcore_uart();
                reflashes += 1;
                last_edge = rtc.count();
                resets = 0;
            } else {
                // Exhausted: keep listening, stop recovering. Latch so this
                // branch does not re-evaluate the timeout every iteration.
                recovery_given_up = true;
            }
        }
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
    /// Channel 1: cellcore UPDI (8E2).
    fn cellcore_updi(&mut self) {
        let _ = self.a1.set_low();
        let _ = self.a0.set_high();
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
    #[expect(clippy::empty_loop)]
    loop {}
}
