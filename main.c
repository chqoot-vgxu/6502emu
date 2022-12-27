#include <stdint.h>

#include <avr/pgmspace.h>

#include "opcodes.h"

#define exec_opcode(OPC) ((opcode_fn) pgm_read_ptr(&opcode_table[OPC]))()

int main(void) {
    init();

    for (;;) {
        opcode_t opc = read_next_opcode();

        exec_opcode(opc);

#if ENABLE_INTERRUPTS != 0
        handle_interrupts();
#endif
    }
}
