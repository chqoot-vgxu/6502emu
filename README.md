# 6502emu
## What it is
A small 6502 emulator running inside an arduino.

## What it's not
A drop in replacement for a real 6502 implemented in an arduino.
The emulator itself gives almost complete control to the internal hardware, so it is possible to use gpios, usart, spi timers, ecc. from the 6502 assembly. 

## Why?
Why not?

## Status
I didn't test this on anything but an atmega328p.

All w65C02 are implemented. (I haven't tested all of them yet)

Interrupts are working. (I haven't tested all of them yet)

I'll write tests eventually.

## Some technical stuff
### Address space
```
  6502                 atmega328p
0xffff  +-----------+  wherever gcc puts rom + rom size (8k in this example)
        |  ROM_END  |
        |           |
        |           |
0xe000  | ROM_START |
0xdfff  +-----------+  wherever gcc puts rom
0x0900  |           |
        | RESERVED1 |
        |           |
0x08ff  +-----------+  0x00ff
        |  REG_END  |
        |           |
0x0800  | REG_START |  0x0000
0x07ff  +-----------+  0x08ff
0x07f0  | RESERVED0 |  0x08f0
0x07ef  +-----------+  0x08ef
        |  RAM_END  |
        |           |
        | RAM_START |
0x0000  +-----------+  0x0100
```

Not all the internal ram is accessible because some functions and IRQs do use some stack space.
This should be less than 16 bytes though, and is the region marked as RESERVED0.

RESERVED1 on the other hand is an empty region.
This region could be used to map emulated peripherals, such as a math coprocessor, to
speed up operations like multiplication and division on integer and floating point numbers, or
a video chip to drive an external display more efficiently.
For example, addresses 0x0900-0x0919 are used for a virtual interrupt controller, reading 0x0900-0x0918
clears the corresponding interrupt flag. Reading 0x0919 returns the interrupt index, which can be used
to index into a table of routines.
```
irq:
    phx
    ldx $0919
    jmp (irq_table,x)

irq_table:
        .word INT1_vect
        .word PCINT0_vect
        .word PCINT1_vect
        .word PCINT2_vect
        .word WDT_vect
        .word TIMER2_COMPA_vect
        .word TIMER2_COMPB_vect
        .word TIMER2_OVF_vect
        .word TIMER1_CAPT_vect
        .word TIMER1_COMPA_vect
        .word TIMER1_COMPB_vect
        .word TIMER1_OVF_vect
        .word TIMER0_COMPA_vect
        .word TIMER0_COMPB_vect
        .word TIMER0_OVF_vect
        .word SPI_STC_vect
        .word USART_RX_vect
        .word USART_UDRE_vect
        .word USART_TX_vect
        .word ADC_vect
        .word EE_READY_vect
        .word ANALOG_COMP_vect
        .word TWI_vect
```

Read to reserved regions will always return 0, writes will be silently ignored.

Care should be taken when writing to registers as some of them are used by the emulator to work properly.
For example addresses $0800-$0820, $083d-$08ef should not be written as they contain the avr architectural registers r0-r31,
the stack pointer and status register.


### Interrupts (WIP)
Not fully tested but seems to be working.

Interrupts can be disabled setting ENABLE_INTERRUPTS=0. This can be useful if interrupts aren't used to reduce
binary size and gain a small speed improvement.

### NMI
The Non Maskable Interrupt is triggered by a falling edge on INT0.

### IRQ/BRK
All other AVR interrupts write To the GPIOR registers and will be checked after each instruction completes.
To clear the interrupt the appropriate flag in the GPIOR registers must be cleared.

### RESET
The only way to reset the emulator is to reset the host.
The ram content won't change between resets but all peripherals are reset and must be reinitialized.
An alternative would be to use an indirect jump to jump to the reset vector.
