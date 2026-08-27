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
    /// SPI0, parked here between runs so each run re-inits it from scratch.
    pub spi0: Option<pac::SPI0>,
    /// App staging EEPROM chip select (PG6, active low).
    pub cs_app: Output,
    /// Current main-clock frequency. USART re-init derives its divisor from
    /// this.
    pub f_cpu: HfFreq,
    /// Whether `clock-extclk` switched to the external clock this boot.
    pub clock_switched: bool,
}
