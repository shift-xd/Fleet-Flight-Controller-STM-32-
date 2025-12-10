/* Very small placeholder startup - replace with vendor startup for interrupts */
.section .text
.global Reset_Handler
Reset_Handler:
    /* Normally: set up stack, call SystemInit, then main */
    bl main
    b .
