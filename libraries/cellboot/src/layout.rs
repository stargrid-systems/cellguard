//! Storage geometry shared by all `CellGuard` firmware images.
//!
//! Three binaries (cellcore app, cellboot bootloader, cellprog programmer)
//! read and write the same EEPROMs. These constants are the single source of
//! truth: a copy in any firmware silently corrupts the others' reads.

/// AVR128DA64 boot section size (FUSE.BOOTSIZE = 16, units of 512 bytes).
/// The bootloader occupies flash below this address. The application starts
/// here.
pub const BOOT_SECTION_SIZE: u32 = 16 * 512;

/// App staging EEPROM capacity (U104, CAT25M01, 128 KB).
pub const APP_EEPROM_CAP: u32 = 128 * 1024;
/// Boot staging EEPROM capacity (U105, CAT25128, 16 KB).
pub const BOOT_EEPROM_CAP: u32 = 16 * 1024;
/// Factory EEPROM capacity (U106, CAT25128, 16 KB). Cellcore-only and never
/// a firmware-update target.
pub const FACTORY_EEPROM_CAP: u32 = 16 * 1024;
/// Cellagent band capacity, carved from the end of the app EEPROM.
pub const CELLAGENT_CAP: u32 = 4 * 1024;
/// Cellprog self-update band capacity, carved from the end of the app
/// EEPROM, below the cellagent band.
pub const CELLPROG_CAP: u32 = 4 * 1024;

/// Offset of the cellprog band within the app EEPROM.
pub const CELLPROG_OFFSET: u32 = APP_EEPROM_CAP - CELLAGENT_CAP - CELLPROG_CAP;
/// Offset of the cellagent band within the app EEPROM.
pub const CELLAGENT_OFFSET: u32 = APP_EEPROM_CAP - CELLAGENT_CAP;
/// Start of the boot EEPROM band in the banded app+boot address space.
pub const BOOT_BAND_OFFSET: u32 = APP_EEPROM_CAP;

/// On-chip EEPROM slot holding the probe-able agent state.
pub const STATE_OFFSET: u16 = 0;
/// Length of the agent state slot.
pub const STATE_LEN: u16 = 64;
/// On-chip EEPROM offset of the panic record (after the state slot).
pub const PANIC_OFFSET: u16 = STATE_LEN;

const _: () = assert!(CELLAGENT_CAP < APP_EEPROM_CAP);
const _: () = assert!(CELLPROG_CAP < APP_EEPROM_CAP);
// The bands must not overlap and each must sit on a CAT25M01 write-page
// boundary, so a staged image never straddles a page write.
const _: () = assert!(CELLPROG_OFFSET + CELLPROG_CAP == CELLAGENT_OFFSET);
const _: () = assert!(CELLPROG_OFFSET.is_multiple_of(256));
