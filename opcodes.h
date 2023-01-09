#pragma once

#include <stdint.h>

#include <avr/io.h>
#include <avr/pgmspace.h>

#ifndef ENABLE_INTERRUPTS
#   define ENABLE_INTERRUPTS 1
#endif

typedef uint8_t flags_t;
typedef uint8_t opcode_t;
typedef void (*opcode_fn)();

#define OPCODE_TABLE_SIZE 256
extern const PROGMEM opcode_fn opcode_table[OPCODE_TABLE_SIZE];

extern void init();

extern opcode_t read_next_opcode();

extern void handle_interrupts();
