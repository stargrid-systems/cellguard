/* Application flash starts after the 8 KB boot section (FUSE.BOOTSIZE = 16). */
MEMORY {
  FLASH (rx) : ORIGIN = 0x2000, LENGTH = 120K
  RAM (rwx)  : ORIGIN = 0x4000, LENGTH = 16K
}
