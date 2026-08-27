//! Panic handler: store the location in the resume record, then
//! software-reset, so the next boot banner reports the panic as a deferred
//! result instead of hanging the session.

use core::panic::PanicInfo;

use avr_device::avr128da64 as pac;
use avrxt_hal::rstctrl::RstInstance;

use crate::resume;

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    avr_device::interrupt::disable();
    let (file, line) = info
        .location()
        .map_or(("", 0), |loc| (loc.file(), loc.line()));
    resume::record_panic(file, line);
    // SAFETY: interrupts are off and the device resets on the next line, so
    // the stolen handle never aliases a live driver in any observable way.
    let dp = unsafe { pac::Peripherals::steal() };
    dp.RSTCTRL.software_reset(&dp.CPU)
}
