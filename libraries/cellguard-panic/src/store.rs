//! EEPROM-backed panic storage and the crash-loop reset/halt decision.
//!
//! [`store_and_decide`] is the entry point a panic handler calls: it reads
//! the stored crash-loop counter, writes a record for the current panic, and
//! returns whether to reset or halt.

use core::panic::PanicInfo;

use avrxt_hal::clock::CcpUnlock;
use avrxt_hal::nvmctrl::{Nvm, NvmInstance};

use crate::record::{PanicRecord, RECORD_LEN};

/// Reads the last panic record from the EEPROM slot at `offset`.
///
/// Call before [`clear`]-ing the slot at boot if the record is needed later.
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
/// Call once a boot has proven itself healthy, or after programming fresh
/// application code. Best-effort: a write error is ignored so a failed erase
/// never blocks the boot.
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
/// A blank or corrupt slot counts as zero. Panics 1..=`threshold` reset, the
/// next one halts. Storage errors fall through to a reset so a flaky EEPROM
/// never blocks recovery.
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
        // Still record the latest location with the counter parked at
        // `threshold`.
        record.consecutive_panics = current;
        let _ = nvm.write_eeprom(offset, &record.serialize(), cpu);
        Decision::Halt
    } else {
        record.consecutive_panics = current + 1;
        let _ = nvm.write_eeprom(offset, &record.serialize(), cpu);
        Decision::Reset
    }
}
