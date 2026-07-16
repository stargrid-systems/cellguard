#![no_std]
#![no_main]
#![feature(abi_avr_interrupt)]

//! CellGuard programmer firmware for the ATtiny406 (U1003), the on-board
//! `cellprog` MCU.
//!
//! This MCU reflashes the cellcore (AVR128) over UPDI, reading staged images
//! from the shared SPI EEPROM. It never acts on its own: the cellcore is the
//! sole orchestrator. It stages images, then sends a `ProgProgram` packet over
//! the cellcore's USART3, which reaches this MCU's USART0 through the U1004
//! analog mux (channel 0).
//!
//! USART0 is shared between that UART command link (mux channel 0, 8N1) and the
//! UPDI link (mux channel 1, 8E2). The firmware listens on channel 0, and on a
//! `ProgProgram` it switches USART0 to 8E2 and the mux to channel 1, flashes the
//! staged image, then switches back to channel 0 and emits the `ProgResult`.
//!
//! Pin map (verified, see `scratch/hardware/cellprog-mcu.md`):
//! - USART0 PB2/PB3 -> U1004 mux.
//! - PA3/PA4 = U1004 select A1/A0.
//! - SPI0_ALT (PC0 SCK, PC1 MISO, PC2 MOSI). App EEPROM U104 CS = PA2, Boot
//!   EEPROM U105 CS = PC3. Factory EEPROM U106 is not reachable here.

use avr_device::attiny406 as pac;
use avrxt_hal::clock::{self, ClkPrescaler, TinyBaseFreq};
use avrxt_hal::delay::Delay;
use avrxt_hal::gpio::{Output, Port};
use avrxt_hal::spi::{Prescaler, Spi};
use avrxt_hal::usart::{Frame, Usart};
use cat25::{CAT25128, CAT25M01, Cat25};
use cellboot::drivers::Cat25Store;
use cellboot::io::BandedStore;
use cellprog::supervisor::{ProgLayout, SourceSlot, Supervisor};
use cellprog::writer::UpdiNvmWriter;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::MODE_0;
use embedded_hal_bus::spi::RefCellDevice;
use embedded_io::Write;

use core::cell::RefCell;
use core::panic::PanicInfo;

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

/// Cellcore flash offsets (relative to flash base) for each staged region.
///
/// These are pending the cellcore app/boot partition decision (BOOTSZ fuse and
/// application firmware layout). The bootloader lives at flash 0; the
/// application base will be fixed once the application firmware exists.
const BOOT_TARGET_BASE: u32 = 0;
const APP_TARGET_BASE: u32 = 0;

#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    avr_device::interrupt::disable();
    loop {}
}

#[avr_device::entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().unwrap();

    // Run the main clock at full speed (20 MHz).
    clock::set_main_clock_prescaler(&dp.CPU, &dp.CLKCTRL, PRESCALER);
    let f_cpu = BASE_FREQ.clk_per_hz(PRESCALER);

    let porta = Port::new(dp.PORTA).split();
    let portb = Port::new(dp.PORTB).split();
    let portc = Port::new(dp.PORTC).split();

    // SPI0 on its alternate (PORTC) pins: PC0 SCK, PC1 MISO, PC2 MOSI.
    dp.PORTMUX
        .ctrlb()
        .write(|w| w.spi0().set_bit());
    let _sck = portc.p0.into_output();
    let _miso = portc.p1.into_input();
    let _mosi = portc.p2.into_output();
    let spi = RefCell::new(Spi::new(dp.SPI0, MODE_0, Prescaler::Div16));

    // App and Boot chip-selects (active low, idle high). PC3 also doubles as
    // the SPI hardware SS; CTRLB.SSD keeps it usable as a GPIO CS.
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
    };
    let mut supervisor = Supervisor::<_, 128>::new(store, layout, NODE_ID);

    // USART0 = the shared link. Start it as the 8N1 UART command link on mux
    // channel 0. TxD idle-high, RxD input.
    let _tx = portb.p2.into_output_high();
    let _rx = portb.p3.into_input();
    let mut usart = Usart::builder(dp.USART0, f_cpu)
        .baud(BAUD)
        .frame(Frame::EIGHT_N_1)
        .build()
        .unwrap_or_else(|_| halt());

    // U1004 mux select: A1 = PA3, A0 = PA4. Channel 0 is the cellcore UART.
    let mut mux = MuxSelect {
        a1: porta.p3.into_output(),
        a0: porta.p4.into_output(),
    };
    mux.cellcore_uart();

    loop {
        let Ok(byte) = usart.read_byte() else {
            continue;
        };
        let Some(source) = supervisor.decode(byte) else {
            continue;
        };

        // Switch the shared USART to UPDI: 8E2, mux channel 1.
        usart.set_frame(Frame::EIGHT_E_2);
        mux.cellcore_updi();
        let status = {
            let link = UsartUpdiLink::new(&mut usart);
            let mut writer = UpdiNvmWriter::new(link);
            supervisor.program(source, &mut writer)
        };

        // Switch back to the 8N1 UART command link and reply.
        usart.set_frame(Frame::EIGHT_N_1);
        mux.cellcore_uart();
        if let Some(reply) = supervisor.reply(status) {
            let _ = usart.write_all(reply);
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
}

/// Halts with interrupts disabled.
fn halt() -> ! {
    avr_device::interrupt::disable();
    loop {}
}
