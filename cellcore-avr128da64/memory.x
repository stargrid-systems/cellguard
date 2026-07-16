/* Application flash starts after the 4 KB boot section (FUSE.BOOTSIZE = 8). */
MEMORY {
  FLASH (rx) : ORIGIN = 0x1000, LENGTH = 124K
  RAM (rwx)  : ORIGIN = 0x4000, LENGTH = 16K
}
