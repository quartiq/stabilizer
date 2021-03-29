.cfi_sections .debug_frame

# .thumb
.section .text.pre_init, "ax"
.globl __pre_init
.type __pre_init,%function
.thumb_func
.cfi_startproc
__pre_init:

  # Enable ITCM and DTCM
  ldr r0, =1
  ldr r1, =0xE000EF90
  ldr r2, [r1]
  # Set ITCMCR.EN
  orr r2, r2, r0
  str r2, [r1]
  ldr r1, =0xE000EF94
  ldr r2, [r1]
  # Set DTCMCR.EN
  orr r2, r2, r0
  str r2, [r1]
  dsb
  isb

  # Analogous to cortex-m-rt Reset code for .data copying.
  # Initialise .itcm code. `__sitcm`, `__siitcm`, and `__eitcm` come from the
  # linker script. Copy from r2 into r0 until r0 reaches r1.
  ldr r0,=__sitcm
  ldr r1,=__eitcm
  ldr r2,=__siitcm
1:
  cmp r1, r0
  beq 2f
  # load 1 word from r2 to r3, inc r2
  ldm r2!, {r3}
  # store 1 word from r3 to r0, inc r0
  stm r0!, {r3}
  b 1b
2:
  dsb
  isb
  bx lr
.cfi_endproc
