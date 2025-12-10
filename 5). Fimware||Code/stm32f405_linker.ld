/* Minimal linker script placeholder.
   Replace with an MCU-accurate one before flashing. */
ENTRY(Reset_Handler)
SECTIONS {
  .text : { *(.text*) } > FLASH
  .data : { *(.data*) } > RAM
  .bss  : { *(.bss*)  } > RAM
}
