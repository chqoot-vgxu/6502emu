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

All but 2 instructions are implemented.

Interrupts not implemented yet.

BCD mode not implemented and probably never will.

The missing instructions are
 * BRK
 * SED

Furthermore some things are practicaly impossible to emulate, for example the 6502 interrupt vector. At reset the 6502 would read the address where to start executing code from 0xFFFC-D, but the atmega328p doesn't have this much flash memory. So it starts executing from whenever the compiler places it. Same story for IRQ/BRK and NMI vectors.
