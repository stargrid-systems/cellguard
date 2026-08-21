//! EEPROM-backed panic storage and the crash-loop reset/halt decision.
//!
//! [`store_and_decide`] is the policy entry point a panic handler calls after
//! taking the peripherals. It reads the stored crash-loop counter from the
//! record slot, writes a fresh record for the current panic, and tells the
//! caller whether to reset or halt.

use core::panic::PanicInfo;

use avrxt_hal::clock::CcpUnlock;
use avrxt_hal::nvmctrl::{Nvm, NvmInstance};

use crate::record::{PanicRecord, RECORD_LEN};

/// Reads the last panic record from the EEPROM slot at `offset`, if a valid one
/// is stored.
///
/// Call this once at boot before [`clear`]-ing the slot, so the firmware can
/// cache the record for a later `PanicProbe` response.
pub fn read_panic_record<T: NvmInstance>(nvm: &Nvm<T>, offset: u16) -> Option<PanicRecord> {
    let mut buf = [0u8; RECORD_LEN];
    nvm.read_eeprom(offset, &mut buf).ok()?;
    PanicRecord::parse(&buf).ok()
}

/// What the caller should do after [`store_and_decide`] records the panic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Decision {
    /// Reset the device: the crash-loop limit has not been reached.
    Reset,
    /// Halt: the limit of consecutive panic-resets has been reached.
    Halt,
}

/// Erases the panic record at `offset`, so the next panic starts a fresh
/// crash-loop.
///
/// Call this once a boot has proven itself healthy (past all initialization
/// that could panic), or after programming fresh application code. A blank
/// slot reads back as "no record", so [`store_and_decide`] treats the next
/// panic as the first. Storage is best-effort: a write error is ignored, since
/// a failed erase never blocks the boot.
pub fn clear<T, C>(nvm: &Nvm<T>, cpu: &C, offset: u16)
where
    T: NvmInstance,
    C: CcpUnlock,
{
    let blank = [0xFFu8; RECORD_LEN];
    let _ = nvm.write_eeprom(offset, &blank, cpu);
}

/// Reads the stored crash-loop counter, writes a record for this panic, and
/// returns whether the caller should reset or halt.
///
/// The counter is read from the [`PanicRecord`] at EEPROM `offset`. A blank or
/// corrupt slot counts as zero. The first `threshold` panics reset the device
/// (counter 1..=`threshold`). The next panic returns [`Decision::Halt`]
/// instead. A healthy boot clears the slot so a single transient panic does not
/// accumulate.
///
/// Storage is best-effort: a read or write error falls through to a reset (the
/// counter is treated as zero, the record is written if it can be), so a flaky
/// EEPROM never blocks recovery.
pub fn store_and_decide<T, C>(
    nvm: &Nvm<T>,
    cpu: &C,
    offset: u16,
    threshold: u8,
    reset_flags: u8,
    info: &PanicInfo,
) -> Decision
where
    T: NvmInstance,
    C: CcpUnlock,
{
    let mut buf = [0u8; RECORD_LEN];
    let current = nvm
        .read_eeprom(offset, &mut buf)
        .ok()
        .and_then(|()| PanicRecord::parse(&buf).ok())
        .map_or(0u8, |r| r.consecutive_panics);

    let mut record = PanicRecord::from_panic_info(info, reset_flags);
    if current >= threshold {
        // Park the counter at the threshold and still record the latest
        // location, then leave the decision to the caller (halt).
        record.consecutive_panics = current;
        let _ = nvm.write_eeprom(offset, &record.serialize(), cpu);
        Decision::Halt
    } else {
        record.consecutive_panics = current + 1;
        let _ = nvm.write_eeprom(offset, &record.serialize(), cpu);
        Decision::Reset
    }
}
