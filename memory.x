MEMORY
{
  ITCM   (rwx) : ORIGIN = 0x00000000, LENGTH = 64K
  RAM    (rwx) : ORIGIN = 0x20000000, LENGTH = 127K
  AXISRAM (rwx) : ORIGIN = 0x24000000, LENGTH = 512K
  SRAM1 (rwx) : ORIGIN = 0x30000000, LENGTH = 128K
  SRAM2 (rwx) : ORIGIN = 0x30020000, LENGTH = 128K
  SRAM3 (rwx) : ORIGIN = 0x30040000, LENGTH = 32K
  BACKUPSRAM (rwx) : ORIGIN = 0x38000000, LENGTH = 64K
  RAM_B  (rwx) : ORIGIN = 0x38800000, LENGTH = 4K
  FLASH  (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  FLASH1 (rx)  : ORIGIN = 0x08100000, LENGTH = 1024K
  PERSISTENT_RAM : ORIGIN = 0x2001FC00, LENGTH = 1K
}

/*
 * Persistent memory has a u32 bootflag at the beginning and then the remainder is used for
 * persisting panic information between boots.
 */
_bootflag = ORIGIN(PERSISTENT_RAM);
_panic_dump_start = ORIGIN(PERSISTENT_RAM) + 4;
_panic_dump_end = ORIGIN(PERSISTENT_RAM) + LENGTH(PERSISTENT_RAM) - 4;

SECTIONS {
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
  .itcm : ALIGN(8) {
    . = ALIGN(8);
    __sitcm = .;
    *(.itcm .itcm.*);
    . = ALIGN(8);
    __eitcm = .;
  } > ITCM AT>FLASH
  __siitcm = LOADADDR(.itcm);
} INSERT BEFORE .data;

ASSERT(__sitcm % 8 == 0 && __eitcm % 8 == 0, "
BUG(cortex-m-rt): .itcm is not 8-byte aligned");

ASSERT(__siitcm % 4 == 0, "
BUG(cortex-m-rt): the LMA of .itcm is not 4-byte aligned");
