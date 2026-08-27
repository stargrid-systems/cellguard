//! Shared runner state passed to every test.

use avr_device::avr128da64 as pac;
use avrxt_hal::clock::HfFreq;
use avrxt_hal::gpio::Output;

use crate::console::Console;

/// Peripheral handles and clock state shared by the runner and the tests.
pub struct Context {
    /// The debug console. Tests use it for payload I/O and log lines.
    pub console: Console,
    /// CPU handle for CCP unlocks (clock switch, watchdog, software reset).
    pub cpu: pac::CPU,
    /// Clock controller, owned by the `clock-extclk` test.
    pub clkctrl: pac::CLKCTRL,
    /// Pin router. The TWI tests set the TWI1 routing on every run.
    pub portmux: pac::PORTMUX,
    /// SPI0, parked here between runs so each run re-inits it from scratch.
    pub spi0: Option<pac::SPI0>,
    /// TWI1, parked here between runs so each run re-inits it from scratch.
    pub twi1: Option<pac::TWI1>,
    /// App staging EEPROM chip select (PG6, active low).
    pub cs_app: Output,
    /// Boot EEPROM chip select (PA7, active low).
    pub cs_boot: Output,
    /// Factory identity EEPROM chip select (PG7, active low).
    pub cs_ident: Output,
    /// Current main-clock frequency. USART re-init derives its divisor from
    /// this.
    pub f_cpu: HfFreq,
    /// Whether `clock-extclk` switched to the external clock this boot.
    pub clock_switched: bool,
}
