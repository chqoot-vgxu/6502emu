#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#include "opcodes.h"

#include "rom.h"

#define AVR_STACK_SIZE 16
#define AVR_REG_SIZE 256
#define RAM_START ((uint16_t)0x0100)
#define REG_START ((uint16_t)0x0800)
#define ROM_START ((uint16_t)0xffff - sizeof(rom) + 1)

#define STACK ((uint8_t*)0x02FF)
#define RAM   ((uint8_t*)0x0100)
#define REG   ((uint8_t*)0x0000)

register uint8_t  sp  asm("r9");
register uint16_t ip  asm("r10");
register uint8_t  ipl asm("r10");
register uint8_t  iph asm("r11");
register uint8_t  a   asm("r12");
register uint8_t  p   asm("r13");
register uint8_t  x   asm("r15");
register uint8_t  y   asm("r14");


#define WORD(h, l) (uint16_t)(((h) << 8) | (l))
#define SET_IP(n) ip  = (n)
#define INC_IP(n) ip += (n)

#define ADC_(t)    set_sreg_c_flag(); asm("adc %0, %1" : "+r"(a) : "r"((t)) : "cc")
#define SBC_(t)    set_sreg_c_flag(); asm("sbc %0, %1" : "+r"(a) : "r"((t)) : "cc")
#define ROL_(t)    set_sreg_c_flag(); asm("rol %0" : "+r"((t)) :: "cc")
#define ROR_(t)    set_sreg_c_flag(); asm("ror %0" : "+r"((t)) :: "cc")
#define CMP_(a, b) asm("cp %0, %1" :: "r" ((a)), "r" ((b)) : "cc")
#define BIT_(t)    asm("mov __tmp_reg__, %0"       "\n\t"   \
                       "and __tmp_reg__, %1"       "\n\t"   \
                       :: "r"(t), "r"(a) : "cc")

uint8_t memory_read(uint16_t addr) {
    // ROM
    if (addr >= ROM_START) {
        return pgm_read_byte(rom + (addr - ROM_START));
    }
    // RAM
    if (addr < REG_START - AVR_STACK_SIZE) {
        return RAM[addr];
    }
    // REG
    if (addr >= REG_START && addr < REG_START + AVR_REG_SIZE) {
        return REG[addr & 0x00ff];
    }

    // RESERVED
    return 0;
}

void memory_write(uint16_t addr, uint8_t v) {
    // RAM
    if (addr < REG_START - AVR_STACK_SIZE) {
        RAM[addr] = v;
    }
    // REG
    else if (addr >= REG_START && addr < REG_START + AVR_REG_SIZE) {
        REG[addr & 0x00ff] = v;
    }
    // RESERVED or ROM
}

void push(uint8_t reg) {
    RAM[--sp] = reg;
}

uint8_t pop() {
    return RAM[sp++];
}


/* SREG to 6502 Flags translation
F <-> SREG
0  C  0
1  Z  1
2  I  different meaning
3  D  no equivalent
4  B  weird bit
5  s  other weird bit
6  V  3
7  N  2
*/

#define FLAG_C 0
#define FLAG_Z 1
#define FLAG_I 2
#define FLAG_D 3
#define FLAG_B 4
#define FLAG_V 6
#define FLAG_N 7

#define SET_FLAG(F) p |=  (1 << FLAG_##F)
#define CLR_FLAG(F) p &= ~(1 << FLAG_##F)
#define FLAG_IS_SET(F) (p & (1 << FLAG_##F))
#define SET_FLAG_ON(C, F) if (C) SET_FLAG(F)
#define SET_FLAG_FROM_SREG(F) SET_FLAG_ON(sreg & (1 << SREG_##F), F)

void update_flags(void) {
    uint8_t sreg = SREG;
    p &= 0b00111100;
    SET_FLAG_FROM_SREG(C);
    SET_FLAG_FROM_SREG(Z);
    SET_FLAG_FROM_SREG(V);
    SET_FLAG_FROM_SREG(N);
}

void update_flags_on_load(int8_t reg) {
    p &= 0b01111101;
    SET_FLAG_ON(reg < 0, N);
    SET_FLAG_ON(reg == 0, Z);
}

void update_flags_on_bit(int8_t t) {
    p &= 0b00111101;
    SET_FLAG_ON((a & t) == 0, Z);
    SET_FLAG_ON(t & (1 << 6), V);
    SET_FLAG_ON(t & (1 << 7), N);
}

void set_sreg_c_flag() {
    uint8_t sreg = SREG & ~(1 << SREG_C);
    sreg |= p & (1 << FLAG_C);
    SREG = sreg;
}

uint8_t immediate() {
    uint8_t v = memory_read(ip);
    INC_IP(1);
    return v;
}

uint16_t zero_page() {
    return immediate();
}

uint16_t zero_page_indexed_x() {
    return immediate() + x;
}

uint16_t zero_page_indexed_y() {
    return immediate() + y;
}

uint16_t absolute() {
    uint8_t l = immediate();
    uint8_t h = immediate();
    return WORD(h, l);
}

uint16_t absolute_indexed_x() {
    return absolute() + x;
}

uint16_t absolute_indexed_y() {
    return absolute() + y;
}

uint16_t indexed_indirect() {
    uint8_t z = immediate() + x;
    uint16_t i = WORD(RAM[z+1], RAM[z]);
    return WORD(memory_read(i+1), memory_read(i));
}

uint16_t indirect_indexed() {
    uint8_t z = immediate();
    uint8_t l = RAM[z];
    uint8_t h = RAM[z+1];
    return memory_read(WORD(h, l) + y);
}

#pragma region AVR INTERRUPTS
#define STRINGHIFY(X) #X
#define AVR_ISR(V, H, A) _AVR_ISR_IMPL(V, H, (1 << ((V##_num-1) % 8)), A)
#define _AVR_ISR_IMPL(V, H, N, A) \
ISR(V, ISR_NAKED) { \
    __asm__( \
        "push r0"                           "\n\t" \
        "in   r0, __SREG__"                 "\n\t" \
        "push r0"                           "\n\t" \
        "push r24"                          "\n\t" \
        "in   r24, " STRINGHIFY(A)          "\n\t" \
        "sbr  r24, lo8(" STRINGHIFY(N) ")"  "\n\t" \
        "rjmp " STRINGHIFY(H) "\n\t" \
    ); \
}

#define handle_GPIOR(N, A) \
__attribute__((__naked__)) \
void handle_GPIOR##N() { \
    __asm__( \
        "out  " STRINGHIFY(A) ", r24"   "\n\t" \
        "pop  r24"                      "\n\t" \
        "pop r0"                        "\n\t" \
        "out  __SREG__, r0"             "\n\t" \
        "pop r0"                        "\n\t" \
        "reti" \
    ); \
}

handle_GPIOR(0, 0x1e)
handle_GPIOR(1, 0x2a)
handle_GPIOR(2, 0x2b)

AVR_ISR(INT0_vect,          handle_GPIOR0, 0x1e)
AVR_ISR(INT1_vect,          handle_GPIOR0, 0x1e)
AVR_ISR(PCINT0_vect,        handle_GPIOR0, 0x1e)
AVR_ISR(PCINT1_vect,        handle_GPIOR0, 0x1e)
AVR_ISR(PCINT2_vect,        handle_GPIOR0, 0x1e)
AVR_ISR(WDT_vect,           handle_GPIOR0, 0x1e)
AVR_ISR(TIMER2_COMPA_vect,  handle_GPIOR0, 0x1e)
AVR_ISR(TIMER2_COMPB_vect,  handle_GPIOR0, 0x1e)

AVR_ISR(TIMER2_OVF_vect,    handle_GPIOR1, 0x2a)
AVR_ISR(TIMER1_CAPT_vect,   handle_GPIOR1, 0x2a)
AVR_ISR(TIMER1_COMPA_vect,  handle_GPIOR1, 0x2a)
AVR_ISR(TIMER1_COMPB_vect,  handle_GPIOR1, 0x2a)
AVR_ISR(TIMER1_OVF_vect,    handle_GPIOR1, 0x2a)
AVR_ISR(TIMER0_COMPA_vect,  handle_GPIOR1, 0x2a)
AVR_ISR(TIMER0_COMPB_vect,  handle_GPIOR1, 0x2a)
AVR_ISR(TIMER0_OVF_vect,    handle_GPIOR1, 0x2a)

AVR_ISR(SPI_STC_vect,       handle_GPIOR2, 0x2b)
AVR_ISR(USART_RX_vect,      handle_GPIOR2, 0x2b)
AVR_ISR(USART_UDRE_vect,    handle_GPIOR2, 0x2b)
AVR_ISR(USART_TX_vect,      handle_GPIOR2, 0x2b)
AVR_ISR(ADC_vect,           handle_GPIOR2, 0x2b)
AVR_ISR(EE_READY_vect,      handle_GPIOR2, 0x2b)
AVR_ISR(ANALOG_COMP_vect,   handle_GPIOR2, 0x2b)
AVR_ISR(TWI_vect,           handle_GPIOR2, 0x2b)

#define NMI_TRIGGERED() (GPIOR0 & 1)
#define IRQ_TRIGGERED() (!FLAG_IS_SET(I) && (GPIOR0 | GPIOR1 | GPIOR2))

#pragma endregion

#define NMI_VECT()   pgm_read_word(rom + sizeof(rom) - 6)
#define RESET_VECT() pgm_read_word(rom + sizeof(rom) - 4)
#define IRQ_VECT()   pgm_read_word(rom + sizeof(rom) - 2)

void call_isr(uint16_t addr, uint8_t b_flag) {
    push(iph);
    push(ipl);
    push((p & 0xcf) | b_flag);
    SET_FLAG(I);
    SET_IP(addr);
}

int main() {
    GPIOR0 = 0;
    GPIOR1 = 0;
    GPIOR2 = 0;

    SET_IP(RESET_VECT());
    for (;;) {
        uint8_t instruction = immediate();

        switch (instruction) {
            case NOP: break;

#pragma region ADC
            case ADC_IMM: {
                uint8_t t = immediate();
                ADC_(t);
                update_flags();
            }
            break;

            case ADC_ZPG: {
                uint16_t addr = zero_page();
                uint8_t t = RAM[addr];
                ADC_(t);
                update_flags();
            }
            break;

            case ADC_ZPG_X: {
                uint16_t addr = zero_page_indexed_x();
                uint8_t t = RAM[addr];
                ADC_(t);
                update_flags();
                break;
            }

            case ADC_ABS: {
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                ADC_(t);
                update_flags();
            }
            break;

            case ADC_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = memory_read(addr);
                ADC_(t);
                update_flags();
            }
            break;

            case ADC_ABS_Y: {
                uint16_t addr = absolute_indexed_y();
                uint8_t t = memory_read(addr);
                ADC_(t);
                update_flags();
            }
            break;

            case ADC_IND_X: {
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                ADC_(t);
                update_flags();
            }
            break;

            case ADC_IND_Y: {
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                ADC_(t);
                update_flags();
            }
            break;
#pragma endregion

#pragma region SBC
            case SBC_IMM: {
                uint8_t t = immediate();
                SBC_(t);
                update_flags();
            }
            break;

            case SBC_ZPG: {
                uint8_t t = RAM[zero_page()];
                SBC_(t);
                update_flags();
            }
            break;

            case SBC_ZPG_X: {
                uint8_t t = RAM[zero_page_indexed_x()];
                SBC_(t);
                update_flags();
            }
            break;

            case SBC_ABS: {
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                SBC_(t);
                update_flags();
            }
            break;

            case SBC_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = memory_read(addr);
                SBC_(t);
                update_flags();
            }
            break;

            case SBC_ABS_Y: {
                uint16_t addr = absolute_indexed_y();
                uint8_t t = memory_read(addr);
                SBC_(t);
                update_flags();
            }
            break;

            case SBC_IND_X: {
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                SBC_(t);
                update_flags();
            }
            break;

            case SBC_IND_Y: {
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                SBC_(t);
                update_flags();
            }
            break;
#pragma endregion

#pragma region AND
            case AND_IMM: {
                a &= immediate();
                update_flags();
            }
            break;

            case AND_ZPG: {
                a &= RAM[zero_page()];
                update_flags();
            }
            break;

            case AND_ZPG_X: {
                a &= RAM[zero_page_indexed_x()];
                update_flags();
            }
            break;

            case AND_ABS: {
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                a &= t;
                update_flags();
            }
            break;

            case AND_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = memory_read(addr);
                a &= t;
                update_flags();
            }
            break;

            case AND_ABS_Y: {
                uint16_t addr = absolute_indexed_y();
                uint8_t t = memory_read(addr);
                a &= t;
                update_flags();
            }
            break;

            case AND_IND_X: {
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                a &= t;
                update_flags();
            }
            break;

            case AND_IND_Y: {
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                a &= t;
                update_flags();
            }
            break;
#pragma endregion

#pragma region ORA
            case ORA_IMM: {
                a |= immediate();
                update_flags();
            }
            break;

            case ORA_ZPG: {
                a |= RAM[zero_page()];
                update_flags();
            }
            break;

            case ORA_ZPG_X: {
                a |= RAM[zero_page_indexed_x()];
                update_flags();
            }
            break;

            case ORA_ABS: {
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                a |= t;
                update_flags();
                break;
            }

            case ORA_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = memory_read(addr);
                a |= t;
                update_flags();
            }
            break;

            case ORA_ABS_Y: {
                uint16_t addr = absolute_indexed_y();
                uint8_t t = memory_read(addr);
                a |= t;
                update_flags();
            }
            break;

            case ORA_IND_X: {
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                a |= t;
                update_flags();
            }
            break;

            case ORA_IND_Y: {
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                a |= t;
                update_flags();
            }
            break;
#pragma endregion

#pragma region EOR
            case EOR_IMM: {
                a ^= immediate();
                update_flags();
            }
            break;

            case EOR_ZPG: {
                a ^= RAM[zero_page()];
                update_flags();
            }
            break;

            case EOR_ZPG_X: {
                a ^= RAM[zero_page_indexed_x()];
                update_flags();
            }
            break;

            case EOR_ABS: {
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                a ^= t;
                update_flags();
            }
            break;

            case EOR_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = memory_read(addr);
                a ^= t;
                update_flags();
            }
            break;

            case EOR_ABS_Y: {
                uint16_t addr = absolute_indexed_y();
                uint8_t t = memory_read(addr);
                a ^= t;
                update_flags();
            }
            break;

            case EOR_IND_X: {
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                a ^= t;
                update_flags();
                break;
            }

            case EOR_IND_Y: {
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                a ^= t;
                update_flags();
            }
            break;
#pragma endregion

#pragma region ASL
            case ASL_A: {
                a <<= 1;
                update_flags();
            }
            break;

            case ASL_ZPG: {
                RAM[zero_page()] <<= 1;
                update_flags();
            }
            break;

            case ASL_ZPG_X: {
                RAM[zero_page_indexed_x()] <<= 1;
                update_flags();
            }
            break;

            case ASL_ABS: {
                RAM[absolute()] <<= 1;
                update_flags();
            }
            break;

            case ASL_ABS_X: {
                RAM[absolute_indexed_x()] <<= 1;
                update_flags();
            }
            break;
#pragma endregion

#pragma region LSR
            case LSR_A: {
                a >>= 1;
                update_flags();
            }
            break;

            case LSR_ZPG: {
                RAM[zero_page()] >>= 1;
                update_flags();
            }
            break;

            case LSR_ZPG_X: {
                RAM[zero_page_indexed_x()] >>= 1;
                update_flags();
            }
            break;

            case LSR_ABS: {
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                memory_write(addr, t >> 1);
                update_flags();
            }
            break;

            case LSR_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = memory_read(addr);
                memory_write(addr, t >> 1);
                update_flags();
            }
            break;
#pragma endregion

#pragma region ROL
            case ROL_A: {
                ROL_(a);
                update_flags();
            }
            break;

            case ROL_ZPG: {
                uint16_t addr = zero_page();
                uint8_t t = RAM[addr];
                ROL_(t);
                update_flags();
                RAM[addr] = t;
            }
            break;

            case ROL_ZPG_X: {
                uint16_t addr = zero_page_indexed_x();
                uint8_t t = RAM[addr];
                ROL_(t);
                update_flags();
                RAM[addr] = t;
            }
            break;

            case ROL_ABS: {
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                ROL_(t);
                update_flags();
                memory_write(addr, t);
            }
            break;

            case ROL_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = memory_read(addr);
                ROL_(t);
                update_flags();
                memory_write(addr, t);
            }
            break;
#pragma endregion

#pragma region ROR
            case ROR_A: {
                ROR_(a);
                update_flags();
            }
            break;

            case ROR_ZPG: {
                uint16_t addr = zero_page();
                uint8_t t = RAM[addr];
                ROR_(t);
                update_flags();
                RAM[addr] = t;
            }
            break;

            case ROR_ZPG_X: {
                uint16_t addr = zero_page_indexed_x();
                uint8_t t = RAM[addr];
                ROR_(t);
                update_flags();
                RAM[addr] = t;
            }
            break;

            case ROR_ABS: {
                uint16_t addr = absolute();
                uint8_t t = RAM[addr];
                ROR_(t);
                update_flags();
                memory_write(addr, t);
            }
            break;

            case ROR_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = RAM[addr];
                ROR_(t);
                update_flags();
                memory_write(addr, t);
            }
            break;
#pragma endregion

#pragma region BIT
            case BIT_ZPG: {
                uint8_t t = RAM[zero_page()];
                BIT_(t);
                update_flags_on_bit(t);
            }
            break;

            case BIT_ABS: {
                uint8_t t = memory_read(absolute());
                BIT_(t);
                update_flags_on_bit(t);
            }
            break;
#pragma endregion

#pragma region BRANCH
            case BPL: {
                int8_t o = immediate();
                if (!FLAG_IS_SET(N)) {
                    INC_IP(o);
                }
            }
            break;

            case BMI: {
                int8_t o = immediate();
                if (FLAG_IS_SET(N)) {
                    INC_IP(o);
                }
            }
            break;

            case BVC: {
                int8_t o = immediate();
                if (!FLAG_IS_SET(V)) {
                    INC_IP(o);
                }
            }
            break;

            case BVS: {
                int8_t o = immediate();
                if (FLAG_IS_SET(V)) {
                    INC_IP(o);
                }
            }
            break;

            case BCC: {
                int8_t o = immediate();
                if (!FLAG_IS_SET(C)) {
                    INC_IP(o);
                }
            }
            break;

            case BCS: {
                int8_t o = immediate();
                if (FLAG_IS_SET(C)) {
                    INC_IP(o);
                }
                break;
            }

            case BNE: {
                int8_t o = immediate();
                if (!FLAG_IS_SET(Z)) {
                    INC_IP(o);
                }
            }
            break;

            case BEQ: {
                int8_t o = immediate();
                if (FLAG_IS_SET(Z)) {
                    INC_IP(o);
                }
            }
            break;
#pragma endregion

#pragma region JUMP
            case JMP_ABS: {
                uint16_t d = absolute();
                SET_IP(d);
            }
            break;

            case JMP_IND: {
                uint16_t addr = absolute();
                ipl = memory_read(addr);
                iph = memory_read(addr + 1);
            }
            break;

            case JSR: {
                uint16_t d = absolute();
                push(iph);
                push(ipl);
                SET_IP(d);
            }
            break;
#pragma endregion

#pragma region RETURN
            case RTS: {
                ipl = pop();
                iph = pop();
            }
            break;

            case RTI: {
                p = pop();
                ipl = pop();
                iph = pop();
            }
            break;
#pragma endregion

#pragma region BRK
            case BRK: {
                INC_IP(1);
                call_isr(IRQ_VECT(), 0x30);
            }
            break;
#pragma endregion

#pragma region CMP
            case CMP_IMM: {
                uint8_t t = immediate();
                CMP_(a, t);
                update_flags();
            }
            break;

            case CMP_ZPG: {
                uint8_t t = RAM[zero_page()];
                CMP_(a, t);
                update_flags();
            }
            break;

            case CMP_ZPG_X: {
                uint8_t t = RAM[zero_page_indexed_x()];
                CMP_(a, t);
                update_flags();
            }
            break;

            case CMP_ABS: {
                uint8_t t = memory_read(absolute());
                CMP_(a, t);
                update_flags();
            }
            break;

            case CMP_ABS_X: {
                uint8_t t = memory_read(absolute_indexed_x());
                CMP_(a, t);
                update_flags();
            }
            break;

            case CMP_ABS_Y: {
                uint8_t t = memory_read(absolute_indexed_y());
                CMP_(a, t);
                update_flags();
            }
            break;

            case CMP_IND_X: {
                uint8_t t = memory_read(indexed_indirect());
                CMP_(a, t);
                update_flags();
            }
            break;

            case CMP_IND_Y: {
                uint8_t t = memory_read(indirect_indexed());
                CMP_(a, t);
                update_flags();
            }
            break;
#pragma endregion

#pragma region CPX
            case CPX_IMM: {
                uint8_t t = immediate();
                CMP_(x, t);
                update_flags();
            }
            break;

            case CPX_ZPG: {
                uint8_t t = RAM[zero_page()];
                CMP_(x, t);
                update_flags();
            }
            break;

            case CPX_ABS: {
                uint8_t t = memory_read(absolute());
                CMP_(x, t);
                update_flags();
            }
            break;
#pragma endregion

#pragma region CPY
            case CPY_IMM: {
                uint8_t t = immediate();
                CMP_(y, t);
                update_flags();
            }
            break;

            case CPY_ZPG: {
                uint8_t t = RAM[zero_page()];
                CMP_(y, t);
                update_flags();
            }
            break;

            case CPY_ABS: {
                uint8_t t = memory_read(absolute());
                CMP_(y, t);
                update_flags();
            }
            break;
#pragma endregion

#pragma region DEC
            case DEC_ZPG: {
                RAM[zero_page()]--;
                update_flags();
            }
            break;

            case DEC_ZPG_X: {
                RAM[zero_page_indexed_x()]--;
                update_flags();
            }
            break;

            case DEC_ABS: {
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                t--;
                update_flags();
                memory_write(addr, t);
            }
            break;

            case DEC_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = memory_read(addr);
                t--;
                update_flags();
                memory_write(addr, t);
            }
            break;

            case DEX: {
                x--;
            }
            break;

            case DEY: {
                y--;
            }
            break;
#pragma endregion

#pragma region INC
            case INC_ZPG: {
                RAM[zero_page()]++;
                update_flags();
            }
            break;

            case INC_ZPG_X: {
                RAM[zero_page_indexed_x()]++;
                update_flags();
            }
            break;

            case INC_ABS: {
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                t++;
                update_flags();
                memory_write(addr, t);
            }
            break;

            case INC_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                uint8_t t = memory_read(addr);
                t++;
                update_flags();
                memory_write(addr, t);
            }
            break;

            case INX: {
                x++;
            }
            break;

            case INY: {
                y++;
            }
            break;
#pragma endregion

#pragma region FLAGS
            case CLC: {
                CLR_FLAG(C);
            }
            break;

            case SEC: {
                SET_FLAG(C);
            }
            break;

            case CLI: {
                sei(); // CLI and SEI work the other way around in the 6502
                CLR_FLAG(I);
            }
            break;

            case SEI: {
                cli();
                SET_FLAG(I);
            }
            break;

            case CLV: {
                CLR_FLAG(V);
            }
            break;

            case CLD:
                break;

            case SED:
                break;
#pragma endregion

#pragma region LDA
            case LDA_IMM: {
                a = immediate();
                update_flags_on_load(a);
            }
            break;

            case LDA_ZPG: {
                a = RAM[zero_page()];
                update_flags_on_load(a);
            }
            break;

            case LDA_ZPG_X: {
                a = RAM[zero_page_indexed_x()];
                update_flags_on_load(a);
            }
            break;

            case LDA_ABS: {
                uint16_t addr = absolute();
                a = memory_read(addr);
                update_flags_on_load(a);
            }
            break;

            case LDA_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                a = memory_read(addr);
                update_flags_on_load(a);
            }
            break;

            case LDA_ABS_Y: {
                uint16_t addr = absolute_indexed_y();
                a = memory_read(addr);
                update_flags_on_load(a);
            }
            break;

            case LDA_IND_X: {
                uint16_t addr = indexed_indirect();
                a = memory_read(addr);
                update_flags_on_load(a);
            }
            break;

            case LDA_IND_Y: {
                uint16_t addr = indirect_indexed();
                a = memory_read(addr);
                update_flags_on_load(a);
            }
            break;
#pragma endregion

#pragma region LDX
            case LDX_IMM: {
                x = immediate();
                update_flags_on_load(x);
            }
            break;

            case LDX_ZPG: {
                x = RAM[zero_page()];
                update_flags_on_load(x);
            }
            break;

            case LDX_ZPG_Y: {
                x = RAM[zero_page_indexed_y()];
                update_flags_on_load(x);
            }
            break;

            case LDX_ABS: {
                uint16_t addr = absolute();
                x = memory_read(addr);
                update_flags_on_load(x);
            }
            break;

            case LDX_ABS_Y: {
                uint16_t addr = absolute_indexed_y();
                x = memory_read(addr);
                update_flags_on_load(x);
            }
            break;
#pragma endregion

#pragma region LDY
            case LDY_IMM: {
                y = immediate();
                update_flags_on_load(y);
            }
            break;

            case LDY_ZPG: {
                y = RAM[zero_page()];
                update_flags_on_load(y);
            }
            break;

            case LDY_ZPG_X: {
                y = RAM[zero_page_indexed_x()];
                update_flags_on_load(y);
            }
            break;

            case LDY_ABS: {
                uint16_t addr = absolute();
                y = memory_read(addr);
                update_flags_on_load(y);
            }
            break;

            case LDY_ABS_X: {
                uint16_t addr = absolute_indexed_x();
                y = memory_read(addr);
                update_flags_on_load(y);
            }
            break;
#pragma endregion

#pragma region STA
            case STA_ZPG: {
                RAM[zero_page()] = a;
            }
            break;

            case STA_ZPG_X: {
                RAM[zero_page_indexed_x()] = a;
            }
            break;

            case STA_ABS: {
                memory_write(absolute(), a);
            }
            break;

            case STA_ABS_X: {
                memory_write(absolute_indexed_x(), a);
            }
            break;

            case STA_ABS_Y: {
                memory_write(absolute_indexed_y(), a);
            }
            break;

            case STA_IND_X: {
                memory_write(indexed_indirect(), a);
            }
            break;

            case STA_IND_Y: {
                memory_write(indirect_indexed(), a);
            }
            break;
#pragma endregion

#pragma region STX
            case STX_ZPG: {
                RAM[zero_page()] = x;
            }
            break;

            case STX_ZPG_Y: {
                RAM[zero_page_indexed_y()] = x;
            }
            break;

            case STX_ABS: {
                memory_write(absolute(), x);
            }
            break;
#pragma endregion

#pragma region STY
            case STY_ZPG: {
                RAM[zero_page()] = y;
            }
            break;

            case STY_ZPG_X: {
                RAM[zero_page_indexed_x()] = y;
            }
            break;

            case STY_ABS: {
                memory_write(absolute(), y);
            }
            break;
#pragma endregion

#pragma region TRANSFERS
            case TAX: {
                x = a;
            }
            break;

            case TXA: {
                a = x;
            }
            break;

            case TAY: {
                y = a;
            }
            break;

            case TYA: {
                a = y;
            }
            break;

            case TXS: {
                sp = x;
            }
            break;

            case TSX: {
                x = sp;
            }
            break;
#pragma endregion

#pragma region STACK OPERATIONS
            case PHA: {
                push(a);
            }
            break;

            case PLA: {
                a = pop();
            }
            break;

            case PHP: {
                push(p | 0x30);
            }
            break;

            case PLP: {
                p = pop();
            }
            break;
#pragma endregion

            default:
                exit(instruction);
        }

        if (NMI_TRIGGERED()) { // edge triggered
            GPIOR0 &= 0b11111110; // clear request
            call_isr(NMI_VECT(), 0x20);
        }

        if (IRQ_TRIGGERED()) { // level triggered
            call_isr(IRQ_VECT(), 0x20);
        }
    }
}
