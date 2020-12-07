MEMORY
{
  ITCM   (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
  RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
  AXISRAM (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
  SRAM1 (rwx) : ORIGIN = 0x30000000, LENGTH = 128K
  SRAM2 (rwx) : ORIGIN = 0x30020000, LENGTH = 128K
  SRAM3 (rwx) : ORIGIN = 0x30040000, LENGTH = 32K
  BACKUPSRAM (rwx) : ORIGIN = 0x38000000, LENGTH = 64K
  RAM_B  (rwx) : ORIGIN = 0x38800000, LENGTH = 4K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  FLASH1 (rx)  : ORIGIN = 0x08100000, LENGTH = 1024K
}

SECTIONS {
  .itcm : ALIGN(8) {
    *(.itcm .itcm.*);
    . = ALIGN(8);
    } > ITCM
  .axisram (NOLOAD) : ALIGN(8) {
    *(.axisram .axisram.*);
    . = ALIGN(8);
    } > AXISRAM
  .sram1 (NOLOAD) : ALIGN(4) {
    *(.sram1 .sram1.*);
    . = ALIGN(4);
    } > SRAM1
  .sram2 (NOLOAD) : ALIGN(4) {
    *(.sram2 .sram2.*);
    . = ALIGN(4);
    } > SRAM2
  .sram3 (NOLOAD) : ALIGN(4) {
    *(.sram3 .sram3.*);
    . = ALIGN(4);
    } > SRAM3
} INSERT AFTER .bss;
