//! Storage geometry shared by all `CellGuard` firmware images.
//!
//! Three independent binaries (cellcore app, cellboot bootloader, cellprog
//! programmer) read and write the same staging EEPROMs and the same on-chip
//! EEPROM slots. The constants here are the single source of truth for that
//! geometry. A copy in any firmware silently corrupts the others' reads, so
//! every image must take them from here.

/// AVR128DA64 boot section size (FUSE.BOOTSIZE = 16, units of 512 bytes).
/// The bootloader occupies flash 0x0000 up to this address; the application
/// starts here.
pub const BOOT_SECTION_SIZE: u32 = 16 * 512;

/// App staging EEPROM capacity (U104, CAT25M01, 128 KB).
pub const APP_EEPROM_CAP: u32 = 128 * 1024;
/// Boot staging EEPROM capacity (U105, CAT25128, 16 KB).
pub const BOOT_EEPROM_CAP: u32 = 16 * 1024;
/// Cellagent band capacity, carved from the end of the app EEPROM.
pub const CELLAGENT_CAP: u32 = 4 * 1024;

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
