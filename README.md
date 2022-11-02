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

All but 2 instructions (SED and CLD) are implemented.

Interrupts are on their way, BK is working already.

BCD mode not implemented and probably never will.

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
0x0900  | RESERVED1 |
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

RESERVED1 on the other hand is an empty region, no memory is present and read or writes are redirected to
the REG region. This region could be used to map emulated peripherals, such as a math coprocessor, to
speed up operation like multiplication and division on integer and floating point numbers, or
a video chip to drive an external display more efficiently.

Read to reserved regions will always return 0, writes are silently ignored.

Care should be taken when writing to registers as some of them are used by the emulator to work properly.
For example addresses $0800-$0820, $083d-$08ef should not be written as they contain the avr architectural registers r0-r31
the avr stack pointer and status register.


### Interrupts (WIP)
Not fully tested but seems to be working.

#### NMI
The Non Maskable Interrupt is triggered by a falling edge on INT0.

#### IRQ/BRK
All other AVR interrupts write To the GPIOR registers and will be checked after each instruction completes.
To clear the interrupt the appropriate flag in te GPIOR registers must be cleared.

#### RESET
The only way to reset the emulator is to reset the host.
The ram content won't change between resets but all peripherals are reset and must be reinitialized.
An alternative would be to use an indirect jump to jump to the reset vector.
