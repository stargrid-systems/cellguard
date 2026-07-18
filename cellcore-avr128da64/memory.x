/* BRING-UP: app at 0x0, no boot section. The bootloader overflows 4 KB and
 * needs a larger BOOTSIZE before it can be used together with the app. */
MEMORY {
  FLASH (rx) : ORIGIN = 0x0000, LENGTH = 128K
  RAM (rwx)  : ORIGIN = 0x4000, LENGTH = 16K
}
