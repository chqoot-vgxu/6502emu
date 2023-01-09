#include <stdint.h>
#include <stdlib.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

#include "opcodes.h"

#define AVR_STACK_SIZE 16
#define AVR_REG_SIZE 256
#define RAM_START ((uint16_t)0x0100)
#define REG_START ((uint16_t)0x0800)
#define ROM_START ((uint16_t)0xffff - sizeof(rom) + 1)
#define STACK     ((volatile uint8_t*)0x0200)

#define AVR_RAM ((volatile uint8_t*)0x0100)
#define AVR_REG ((volatile uint8_t*)0x0000)

#define SET_PC(n) pc  = (n)
#define INC_PC(n) pc += ((uint16_t) n)

typedef __uint24 uint24_t;

register uint24_t iv asm("r6");
register uint8_t  s  asm("r9");
register uint16_t pc asm("r10");
register uint8_t  a  asm("r12");
register flags_t  p  asm("r13");
register uint8_t  x  asm("r14");
register uint8_t  y  asm("r15");

#define low(R)  ((uint8_t)(R & 0xff))
#define high(R) ((uint8_t)(R >> 8))
#define word(h, l) (uint16_t)(((h) << 8) | (l))

// #region flag helpers

// SREG to 6502 Flags translation
//          F <->   SREG
#define FLAG_C 0 // C  0
#define FLAG_Z 1 // Z  1
#define FLAG_I 2 // I  different meaning
#define FLAG_D 3 // D  no equivalent
#define FLAG_B 4 // B  weird bit
                 // s  other weird bit
#define FLAG_V 6 // V  3
#define FLAG_N 7 // N  2

#define SET_FLAG(F) p |= (uint8_t)  (1 << FLAG_##F)
#define CLR_FLAG(F) p &= (uint8_t) ~(1 << FLAG_##F)
#define FLAG_IS_SET(F) (p & (1 << FLAG_##F))

#define INTERRUPTS_ENABLED() !FLAG_IS_SET(I)

// #endregion

// #region addressing mode helpers

static uint8_t memory_read(uint16_t addr) {
    // ROM
    if (addr >= ROM_START) {
        return pgm_read_byte(rom - ROM_START + addr);
    }
    // RAM
    if (addr < REG_START - AVR_STACK_SIZE) {
        return AVR_RAM[addr];
    }
    // REG
    if (addr < REG_START + AVR_REG_SIZE) {
        return AVR_REG[addr & 0x00ff];
    }

    // virtual registers
#if ENABLE_INTERRUPTS != 0 // virtual irq controller

    if (addr < 0x0918) {
        // clear interrupt
        uint8_t a = (addr & 0x001f);
        iv &= ~((uint24_t)1 << a);
        return 0;
    }

    if (addr == 0x0918) {
        // return interrupt index
        uint24_t vec = iv;
        for (uint8_t i = 0; i < 24; i++) {
            if (vec & 1) {
                return (i - 1) * 2;
            }
            vec >>= 1;
        }
    }

#endif

    // RESERVED
    return 0;
}

static void memory_write(uint16_t addr, uint8_t v) {
    // RAM
    if (addr < REG_START - AVR_STACK_SIZE) {
        AVR_RAM[addr] = v;
    }
    // REG
    else if (addr >= REG_START && addr < REG_START + AVR_REG_SIZE) {
        AVR_REG[addr & 0x00ff] = v;
    }
    // RESERVED or ROM
}

static void push(uint8_t reg) {
    STACK[s--] = reg;
}

static uint8_t pop() {
    return STACK[++s];
}

static uint8_t immediate() {
    uint8_t v = memory_read(pc);
    INC_PC(1);
    return v;
}

static uint16_t zero_page() {
    return immediate();
}

static uint16_t zero_page_indexed_x() {
    return (uint8_t)(immediate() + x);
}

static uint16_t zero_page_indexed_y() {
    return (uint8_t)(immediate() + y);
}

static uint16_t absolute() {
    uint8_t l = immediate();
    uint8_t h = immediate();
    return word(h, l);
}

static uint16_t absolute_indexed_x() {
    return absolute() + x;
}

static uint16_t absolute_indexed_y() {
    return absolute() + y;
}

static uint16_t absolute_indirect() {
    uint16_t addr = absolute();
    return word(memory_read(addr+1), memory_read(addr));
}

static uint16_t absolute_indexed_indirect() {
    uint16_t addr = absolute() + x;
    return word(memory_read(addr+1), memory_read(addr));
}

static uint16_t indirect_zero_page() {
    uint8_t z = immediate();
    return word(AVR_RAM[z+1], AVR_RAM[z]);
}

static uint16_t indexed_indirect() {
    uint8_t z = immediate() + x;
    return word(AVR_RAM[z+1], AVR_RAM[z]);
}

static uint16_t indirect_indexed() {
    uint8_t z = immediate();
    return word(AVR_RAM[z+1], AVR_RAM[z]) + y;
}

// #endregion

// #region interrupt helpers
#define NMI_VECT()   pgm_read_word(rom + sizeof(rom) - 6)
#define RESET_VECT() pgm_read_word(rom + sizeof(rom) - 4)
#define IRQ_VECT()   pgm_read_word(rom + sizeof(rom) - 2)

static void call_isr(uint16_t addr, uint8_t b) {
    push(high(pc));
    push(low(pc));
    push(p | (b << FLAG_B));
    SET_FLAG(I);
    SET_PC(addr);
}

#if ENABLE_INTERRUPTS != 0

#define AVR_ISR(V) _AVR_ISR_IMPL(V, ((uint24_t)1 << ((V##_num-1) % 24)))
#define _AVR_ISR_IMPL(V, N) \
ISR(V, ISR_NAKED) {         \
    iv |= N;                \
    reti();                 \
}

AVR_ISR(INT0_vect)
AVR_ISR(INT1_vect)
AVR_ISR(PCINT0_vect)
AVR_ISR(PCINT1_vect)
AVR_ISR(PCINT2_vect)
AVR_ISR(WDT_vect)
AVR_ISR(TIMER2_COMPA_vect)
AVR_ISR(TIMER2_COMPB_vect)

AVR_ISR(TIMER2_OVF_vect)
AVR_ISR(TIMER1_CAPT_vect)
AVR_ISR(TIMER1_COMPA_vect)
AVR_ISR(TIMER1_COMPB_vect)
AVR_ISR(TIMER1_OVF_vect)
AVR_ISR(TIMER0_COMPA_vect)
AVR_ISR(TIMER0_COMPB_vect)
AVR_ISR(TIMER0_OVF_vect)

AVR_ISR(SPI_STC_vect)
AVR_ISR(USART_RX_vect)
AVR_ISR(USART_UDRE_vect)
AVR_ISR(USART_TX_vect)
AVR_ISR(ADC_vect)
AVR_ISR(EE_READY_vect)
AVR_ISR(ANALOG_COMP_vect)
AVR_ISR(TWI_vect)

#define nmi_triggered() iv & 1
#define irq_triggered() INTERRUPTS_ENABLED() && iv
void handle_interrupts() {
    if (nmi_triggered()) { // edge triggered
        iv &= ~1; // clear request
        call_isr(NMI_VECT(), 0);
    }

    if (irq_triggered()) { // level triggered
        call_isr(IRQ_VECT(), 0);
    }
}
#endif

// #endregion

void init() {
    iv = 0;

    p = 0x20 | (1 << FLAG_I);
    SET_PC(RESET_VECT());

#if ENABLE_INTERRUPTS != 0
    // setup INT0 for emulated NMI
    PORTD = 1 << PD2;
    EICRA = 2 << ISC00;
    EIMSK = 1 << INT0;
    sei();
#else
    cli();
#endif
}

opcode_t read_next_opcode() {
    return (opcode_t) immediate();
}

// #region OPCODES
#define _opcode(name) static void (name)()

// #region helpers
static void _update_flags_abc() {
    uint8_t sreg = SREG;
    asm volatile (
        "mov  r25,%[p]"             "\n\t"
        "cbr  r25,%[cleared_flags]" "\n\t"
        "sbrc %[sreg],%[sreg_c]"    "\n\t"
        "sbr  r25,1 << %[flag_c]"   "\n\t"
        "sbrc %[sreg],%[sreg_z]"    "\n\t"
        "sbr  r25,1 << %[flag_z]"   "\n\t"
        "sbrc %[sreg],%[sreg_v]"    "\n\t"
        "sbr  r25,1 << %[flag_v]"   "\n\t"
        "sbrc %[sreg],%[sreg_n]"    "\n\t"
        "sbr  r25,1 << %[flag_n]"   "\n\t"
        "mov  %[p],r25"
        : [p]"+l"(p)
        : [sreg]"r"(sreg),
          [sreg_c]"M"(SREG_C),
          [sreg_z]"M"(SREG_Z),
          [sreg_v]"M"(SREG_V),
          [sreg_n]"M"(SREG_N),
          [cleared_flags]"M"((1 << FLAG_N) | (1 << FLAG_V) | (1 << FLAG_Z) | (1 << FLAG_C)),
          [flag_c]"M"(FLAG_C),
          [flag_z]"M"(FLAG_Z),
          [flag_v]"M"(FLAG_V),
          [flag_n]"M"(FLAG_N)
    );
}

static void _update_flags_sbc() {
    uint8_t sreg = SREG;
    asm volatile (
        "mov  r25,%[p]"             "\n\t"
        "cbr  r25,%[cleared_flags]" "\n\t"
        "sbrs %[sreg],%[sreg_c]"    "\n\t"
        "sbr  r25,1 << %[flag_c]"   "\n\t"
        "sbrc %[sreg],%[sreg_z]"    "\n\t"
        "sbr  r25,1 << %[flag_z]"   "\n\t"
        "sbrc %[sreg],%[sreg_v]"    "\n\t"
        "sbr  r25,1 << %[flag_v]"   "\n\t"
        "sbrc %[sreg],%[sreg_n]"    "\n\t"
        "sbr  r25,1 << %[flag_n]"   "\n\t"
        "mov  %[p],r25"
        : [p]"+l"(p)
        : [sreg]"r"(sreg),
          [sreg_c]"M"(SREG_C),
          [sreg_z]"M"(SREG_Z),
          [sreg_v]"M"(SREG_V),
          [sreg_n]"M"(SREG_N),
          [cleared_flags]"M"((1 << FLAG_N) | (1 << FLAG_V) | (1 << FLAG_Z) | (1 << FLAG_C)),
          [flag_c]"M"(FLAG_C),
          [flag_z]"M"(FLAG_Z),
          [flag_v]"M"(FLAG_V),
          [flag_n]"M"(FLAG_N)
    );
}

static void _update_flags_NZC() {
    uint8_t sreg = SREG;
    asm volatile (
        "mov  r25,%[p]"             "\n\t"
        "cbr  r25,%[cleared_flags]" "\n\t"
        "sbrc %[sreg],%[sreg_c]"    "\n\t"
        "sbr  r25,1 << %[flag_c]"   "\n\t"
        "sbrc %[sreg],%[sreg_z]"    "\n\t"
        "sbr  r25,1 << %[flag_z]"   "\n\t"
        "sbrc %[sreg],%[sreg_n]"    "\n\t"
        "sbr  r25,1 << %[flag_n]"   "\n\t"
        "mov  %[p],r25"
        : [p]"+l"(p)
        : [sreg]"r"(sreg),
          [sreg_c]"M"(SREG_C),
          [sreg_z]"M"(SREG_Z),
          [sreg_n]"M"(SREG_N),
          [cleared_flags]"M"((1 << FLAG_N) | (1 << FLAG_Z) | (1 << FLAG_C)),
          [flag_c]"M"(FLAG_C),
          [flag_z]"M"(FLAG_Z),
          [flag_n]"M"(FLAG_N)
    );
}

static void _update_flags_NZ() {
    uint8_t sreg = SREG;
    asm volatile (
        "mov  r25,%[p]"             "\n\t"
        "cbr  r25,%[cleared_flags]" "\n\t"
        "sbrc %[sreg],%[sreg_z]"    "\n\t"
        "sbr  r25,1 << %[flag_z]"   "\n\t"
        "sbrc %[sreg],%[sreg_n]"    "\n\t"
        "sbr  r25,1 << %[flag_n]"   "\n\t"
        "mov  %[p],r25"
        : [p]"+l"(p)
        : [sreg]"r"(sreg),
          [sreg_z]"M"(SREG_Z),
          [sreg_n]"M"(SREG_N),
          [cleared_flags]"M"((1 << FLAG_N) | (1 << FLAG_Z)),
          [flag_z]"M"(FLAG_Z),
          [flag_n]"M"(FLAG_N)
    );
}

static void _update_flags_LD(uint8_t v) {
    asm volatile (
        "mov  r25,%[p]"             "\n\t"
        "cbr  r25,%[cleared_flags]" "\n\t"
        "sbrc r24,7"                "\n\t"
        "sbr  r25,1 << %[flag_n]"   "\n\t"
        "cpse r24,r1"               "\n\t"
        "rjmp 1f"                   "\n\t"
        "sbr  r25,1 << %[flag_z]"   "\n\t"
    "1:  mov  %[p],r25"
        : [p]"+l"(p)
        : [v]"r"(v),
          [cleared_flags]"M"((1 << FLAG_N) | (1 << FLAG_Z)),
          [flag_z]"M"(FLAG_Z),
          [flag_n]"M"(FLAG_N)
    );
}

static void daa() {
    uint8_t v = a;
    asm volatile (
        "in   r25,%[sreg]"          "\n\t"
        "cpi  %[v],0x9A"            "\n\t"
        "brlo DAA_lo"               "\n\t"
        "sbr  r25,1 << %[sreg_c]"   "\n"
    "DAA_lo:"                       "\n\t"
        "sbrs r25,%[sreg_h]"        "\n\t"
        "brhs DAA_hi"               "\n\t"
        "subi %[v],-0x06"           "\n"
    "DAA_hi:"                       "\n\t"
        "sbrc r25,%[sreg_c]"        "\n\t"
        "subi %[v],-0x60"           "\n\t"
        "out  %[sreg],r25"          "\n\t"
        "mov  %[a],%[v]"
        : [v]"+r"(v),
          [a]"=l"(a)
        : [sreg]"M"(_SFR_IO_ADDR(SREG)),
          [sreg_c]"M"(SREG_C),
          [sreg_h]"M"(SREG_H)
    );
}

static void das() {
    uint8_t v = a;
    asm volatile (
        "in   r25,%[sreg]"          "\n\t"
        "cpi  %[v],0xA9"            "\n\t"
        "brlo DAS_lo"               "\n\t"
        "sbr  r25,1 << %[sreg_c]"   "\n"
    "DAS_lo:"                       "\n\t"
        "sbrs r25,%[sreg_h]"        "\n\t"
        "brhs DAS_hi"               "\n\t"
        "subi %[v],0x06"            "\n"
    "DAS_hi:"                       "\n\t"
        "sbrc r25,%[sreg_c]"        "\n\t"
        "subi %[v],0x60"            "\n\t"
        "out  %[sreg],r25"          "\n\t"
        "mov  %[a],%[v]"
        : [v]"+r"(v),
          [a]"=l"(a)
        : [sreg]"M"(_SFR_IO_ADDR(SREG)),
          [sreg_c]"M"(SREG_C),
          [sreg_h]"M"(SREG_H)
    );
}

// #endregion

_opcode(nop) {}
_opcode(undefined) __attribute__((alias("nop")));

_opcode(brk) {
    INC_PC(1);
    call_isr(IRQ_VECT(), 1);
}

// #region ADC
static void op_adc(uint8_t v) {
    asm volatile (
        "clc"                       "\n\t"
        "sbrc %[p],%[flag_c]"       "\n\t"
        "sec"                       "\n\t"
        "adc  %[a],%[v]"
        : [a]"+l"(a)
        : [p]"l"(p),
          [v]"r"(v),
          [flag_c]"M"(FLAG_C)
    );
    if (FLAG_IS_SET(D)) {
        daa();
    }
    _update_flags_abc();
}

_opcode(adc_imm) {
    uint8_t t = immediate();
    op_adc(t);
}

_opcode(adc_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    op_adc(t);
}

_opcode(adc_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    uint8_t t = AVR_RAM[addr];
    op_adc(t);
}

_opcode(adc_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    op_adc(t);
}

_opcode(adc_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    op_adc(t);
}

_opcode(adc_abs_y) {
    uint16_t addr = absolute_indexed_y();
    uint8_t t = memory_read(addr);
    op_adc(t);
}

_opcode(adc_ind) {
    uint16_t addr = indirect_zero_page();
    uint8_t t = memory_read(addr);
    op_adc(t);
}

_opcode(adc_x_ind) {
    uint16_t addr = indexed_indirect();
    uint8_t t = memory_read(addr);
    op_adc(t);
}

_opcode(adc_ind_y) {
    uint16_t addr = indirect_indexed();
    uint8_t t = memory_read(addr);
    op_adc(t);
}

// #endregion

// #region SBC
static void op_sbc(uint8_t v) {
    asm volatile (
        "sec"                       "\n\t"
        "sbrc %[p],%[flag_c]"       "\n\t"
        "clc"                       "\n\t"
        "sbc  %[a],%[v]"
        : [p]"+l"(p),
          [a]"+l"(a)
        : [v]"r"(v),
          [flag_c]"M"(FLAG_C)
    );
    if (FLAG_IS_SET(D)) {
        das();
    }
    _update_flags_sbc();
}

_opcode(sbc_imm) {
    uint8_t t = immediate();
    op_sbc(t);
}

_opcode(sbc_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    op_sbc(t);
}

_opcode(sbc_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    uint8_t t = AVR_RAM[addr];
    op_sbc(t);
}

_opcode(sbc_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    op_sbc(t);
}

_opcode(sbc_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    op_sbc(t);
}

_opcode(sbc_abs_y) {
    uint16_t addr = absolute_indexed_y();
    uint8_t t = memory_read(addr);
    op_sbc(t);
}

_opcode(sbc_ind) {
    uint16_t addr = indirect_zero_page();
    uint8_t t = memory_read(addr);
    op_sbc(t);
}

_opcode(sbc_x_ind) {
    uint16_t addr = indexed_indirect();
    uint8_t t = memory_read(addr);
    op_sbc(t);
}

_opcode(sbc_ind_y) {
    uint16_t addr = indirect_indexed();
    uint8_t t = memory_read(addr);
    op_sbc(t);
}

// #endregion

// #region ORA
static void op_ora(uint8_t v) {
    a |= v;
    _update_flags_NZ();
}

_opcode(ora_imm) {
    uint8_t v = immediate();
    op_ora(v);
}

_opcode(ora_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    op_ora(t);
}

_opcode(ora_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    uint8_t t = AVR_RAM[addr];
    op_ora(t);
}

_opcode(ora_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    op_ora(t);
}

_opcode(ora_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    op_ora(t);
}

_opcode(ora_abs_y) {
    uint16_t addr = absolute_indexed_y();
    uint8_t t = memory_read(addr);
    op_ora(t);
}

_opcode(ora_ind) {
    uint16_t addr = indirect_zero_page();
    uint8_t t = memory_read(addr);
    op_ora(t);
}

_opcode(ora_x_ind) {
    uint16_t addr = indexed_indirect();
    uint8_t t = memory_read(addr);
    op_ora(t);
}

_opcode(ora_ind_y) {
    uint16_t addr = indirect_indexed();
    uint8_t t = memory_read(addr);
    op_ora(t);
}

// #endregion

// #region AND
static void op_and(uint8_t v) {
    a &= v;
    _update_flags_NZ();
}

_opcode(and_imm) {
    uint8_t v = immediate();
    op_and(v);
}

_opcode(and_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    op_and(t);
}

_opcode(and_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    uint8_t t = AVR_RAM[addr];
    op_and(t);
}

_opcode(and_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    op_and(t);
}

_opcode(and_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    op_and(t);
}

_opcode(and_abs_y) {
    uint16_t addr = absolute_indexed_y();
    uint8_t t = memory_read(addr);
    op_and(t);
}

_opcode(and_ind) {
    uint16_t addr = indirect_zero_page();
    uint8_t t = memory_read(addr);
    op_and(t);
}

_opcode(and_x_ind) {
    uint16_t addr = indexed_indirect();
    uint8_t t = memory_read(addr);
    op_and(t);
}

_opcode(and_ind_y) {
    uint16_t addr = indirect_indexed();
    uint8_t t = memory_read(addr);
    op_and(t);
}

// #endregion

// #region EOR
static void op_eor(uint8_t v) {
    a ^= v;
    _update_flags_NZ();
}

_opcode(eor_imm) {
    uint8_t v = immediate();
    op_eor(v);
}

_opcode(eor_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    op_eor(t);
}

_opcode(eor_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    uint8_t t = AVR_RAM[addr];
    op_eor(t);
}

_opcode(eor_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    op_eor(t);
}

_opcode(eor_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    op_eor(t);
}

_opcode(eor_abs_y) {
    uint16_t addr = absolute_indexed_y();
    uint8_t t = memory_read(addr);
    op_eor(t);
}

_opcode(eor_ind) {
    uint16_t addr = indirect_zero_page();
    uint8_t t = memory_read(addr);
    op_eor(t);
}

_opcode(eor_x_ind) {
    uint16_t addr = indexed_indirect();
    uint8_t t = memory_read(addr);
    op_eor(t);
}

_opcode(eor_ind_y) {
    uint16_t addr = indirect_indexed();
    uint8_t t = memory_read(addr);
    op_eor(t);
}

// #endregion

// #region INC/DEC

// #region INC/DEC

_opcode(inc_a) {
    a++;
    _update_flags_NZ();
}

_opcode(inc_zpg) {
    uint16_t addr = zero_page();
    AVR_RAM[addr]++;
    _update_flags_NZ();
}

_opcode(inc_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    AVR_RAM[addr]++;
    _update_flags_NZ();
}

#define asm_inc(V) asm(         \
    "subi %[v],-1"      "\n\t"  \
    "call _update_flags_NZ"     \
    : [v]"+r"(V)                \
);

_opcode(inc_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    asm_inc(t);
    memory_write(addr, t);
}

_opcode(inc_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    asm_inc(t);
    memory_write(addr, t);
}

_opcode(dec_a) {
    a--;
    _update_flags_NZ();
}

_opcode(dec_zpg) {
    uint16_t addr = zero_page();
    AVR_RAM[addr]--;
    _update_flags_NZ();
}

_opcode(dec_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    AVR_RAM[addr]--;
    _update_flags_NZ();
}

#define asm_sub(V) asm(         \
    "subi %[v],1"       "\n\t"  \
    "call _update_flags_NZ"     \
    : [v]"+r"(V)                \
);

_opcode(dec_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    asm_sub(t);
    memory_write(addr, t);
}

_opcode(dec_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    asm_sub(t);
    memory_write(addr, t);
}

// #endregion

// #region INX/DEX

_opcode(inx) {
    x++;
    _update_flags_NZ();
}

_opcode(dex) {
    x--;
    _update_flags_NZ();
}

// #endregion

// #region INY/DEY

_opcode(iny) {
    y++;
    _update_flags_NZ();
}

_opcode(dey) {
    y--;
    _update_flags_NZ();
}

// #endregion

// #endregion

// #region ASL

_opcode(asl_a) {
    a <<= 1;
    _update_flags_NZC();
}

_opcode(asl_zpg) {
    uint16_t addr = zero_page();
    AVR_RAM[addr] <<= 1;
    _update_flags_NZC();
}

_opcode(asl_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    AVR_RAM[addr] <<= 1;
    _update_flags_NZC();
}

_opcode(asl_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    t <<= 1;
    _update_flags_NZC();
    memory_write(addr, t);
}

_opcode(asl_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    t <<= 1;
    _update_flags_NZC();
    memory_write(addr, t);
}

// #endregion

// #region LSR

_opcode(lsr_a) {
    a >>= 1;
    _update_flags_NZC();
}

_opcode(lsr_zpg) {
    uint16_t addr = zero_page();
    AVR_RAM[addr] >>= 1;
    _update_flags_NZC();
}

_opcode(lsr_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    AVR_RAM[addr] >>= 1;
    _update_flags_NZC();
}

_opcode(lsr_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    t >>= 1;
    _update_flags_NZC();
    memory_write(addr, t);
}

_opcode(lsr_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    t >>= 1;
    _update_flags_NZC();
    memory_write(addr, t);
}

// #endregion

// #region ROL
static uint8_t op_rol(uint8_t v) {
    asm volatile(
        "clc"                   "\n\t"
        "sbrc %[p],%[flag_c]"   "\n\t"
        "sec"                   "\n\t"
        "rol  %[v]"
        : [v]"+r"(v)
        : [p]"l"(p),
          [flag_c]"M"(FLAG_C)
    );
    _update_flags_NZC();
    return v;
}

_opcode(rol_a) {
    a = op_rol(a);
}

_opcode(rol_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    t = op_rol(t);
    AVR_RAM[addr] = t;
}

_opcode(rol_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    uint8_t t = AVR_RAM[addr];
    t = op_rol(t);
    AVR_RAM[addr] = t;
}

_opcode(rol_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    t = op_rol(t);
    memory_write(addr, t);
}

_opcode(rol_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    t = op_rol(t);
    memory_write(addr, t);
}

// #endregion

// #region ROR
static uint8_t op_ror(uint8_t v) {
    asm volatile(
        "clc"                   "\n\t"
        "sbrc %[p],%[flag_c]"   "\n\t"
        "sec"                   "\n\t"
        "ror  %[v]"
        : [v]"+r"(v)
        : [p]"l"(p),
          [flag_c]"M"(FLAG_C)
    );
    _update_flags_NZC();
    return v;
}

_opcode(ror_a) {
    a = op_ror(a);
}

_opcode(ror_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    t = op_ror(t);
    AVR_RAM[addr] = t;
}

_opcode(ror_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    uint8_t t = AVR_RAM[addr];
    t = op_ror(t);
    AVR_RAM[addr] = t;
}

_opcode(ror_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    t = op_ror(t);
    memory_write(addr, t);
}

_opcode(ror_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    t = op_ror(t);
    memory_write(addr, t);
}

// #endregion

// #region BIT
_opcode(bit_imm) {
    uint8_t t = immediate();
    asm volatile (
        "and  %[v],%[a]"            "\n\t"
        "in   r24,%[sreg]"          "\n\t"
        "mov  r25,%[p]"             "\n\t"
        "cbr  r25,%[cleared_flags]" "\n\t"
        "sbrc r24,%[sreg_z]"        "\n\t"
        "sbr  r25,1 << %[flag_z]"   "\n\t"
        "mov  %[p],r25"
        : [p]"+l"(p),
          [a]"+l"(a)
        : [v]"r"(t),
          [sreg]"I"(_SFR_IO_ADDR(SREG)),
          [sreg_z]"M"(SREG_Z),
          [cleared_flags]"M"((1 << FLAG_N) | (1 << FLAG_V) | (1 << FLAG_Z)),
          [flag_z]"M"(FLAG_Z)
    );
}

static void op_bit(uint8_t v) {
    asm volatile (
        "and  %[v],%[a]"            "\n\t"
        "in   r24,%[sreg]"          "\n\t"
        "mov  r25,%[p]"             "\n\t"
        "cbr  r25,%[cleared_flags]" "\n\t"
        "sbrc r24,%[sreg_z]"        "\n\t"
        "sbr  r25,1 << %[flag_z]"   "\n\t"
        "sbrc %[v],6"               "\n\t"
        "sbr  r25,1 << %[flag_v]"   "\n\t"
        "sbrc %[v],7"               "\n\t"
        "sbr  r25,1 << %[flag_n]"   "\n\t"
        "mov  %[p],r25"
        : [p]"+l"(p),
          [a]"+l"(a)
        : [v]"r"(v),
          [sreg]"I"(_SFR_IO_ADDR(SREG)),
          [sreg_z]"M"(SREG_Z),
          [cleared_flags]"M"((1 << FLAG_N) | (1 << FLAG_V) | (1 << FLAG_Z)),
          [flag_z]"M"(FLAG_Z),
          [flag_v]"M"(FLAG_V),
          [flag_n]"M"(FLAG_N)
    );
}

_opcode(bit_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    op_bit(t);
}

_opcode(bit_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    uint8_t t = AVR_RAM[addr];
    op_bit(t);
}

_opcode(bit_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    op_bit(t);
}

_opcode(bit_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    op_bit(t);
}

static uint8_t op_trb(uint8_t v) {
    uint8_t r;
    asm volatile (
        "and  %[v],%[a]"            "\n\t"
        "in   r24,%[sreg]"          "\n\t"
        "mov  r25,%[p]"             "\n\t"
        "cbr  r25,%[cleared_flags]" "\n\t"
        "sbrc r24,%[sreg_z]"        "\n\t"
        "sbr  r25,1 << %[flag_z]"   "\n\t"
        "mov  %[p],r25"             "\n\t"
        "mov  r25,%[a]"             "\n\t"
        "com  r25"                  "\n\t"
        "and  %[r],r25"
        : [p]"+l"(p),
          [r]"=r"(r)
        : [a]"l"(a),
          [v]"r"(v),
          [sreg]"I"(_SFR_IO_ADDR(SREG)),
          [sreg_z]"M"(SREG_Z),
          [cleared_flags]"M"(1 << FLAG_Z),
          [flag_z]"M"(FLAG_Z)
    );
    return r;
}

_opcode(trb_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    uint8_t r = op_trb(t);
    AVR_RAM[addr] = r;
}

_opcode(trb_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    uint8_t r = op_trb(t);
    memory_write(addr, r);
}

static uint8_t op_tsb(uint8_t v) {
    uint8_t r;
    asm volatile (
        "and  %[v],%[a]"            "\n\t"
        "in   r24,%[sreg]"          "\n\t"
        "mov  r25,%[p]"             "\n\t"
        "cbr  r25,%[cleared_flags]" "\n\t"
        "sbrc r24,%[sreg_z]"        "\n\t"
        "sbr  r25,1 << %[flag_z]"   "\n\t"
        "mov  %[p],r25"             "\n\t"
        "mov  r25,%[a]"             "\n\t"
        "or   %[r],%[a]"
        : [p]"+l"(p),
          [r]"=r"(r)
        : [a]"l"(a),
          [v]"r"(v),
          [sreg]"I"(_SFR_IO_ADDR(SREG)),
          [sreg_z]"M"(SREG_Z),
          [cleared_flags]"M"(1 << FLAG_Z),
          [flag_z]"M"(FLAG_Z)
    );
    return r;
}

_opcode(tsb_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    uint8_t r = op_tsb(t);
    AVR_RAM[addr] = r;
}

_opcode(tsb_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    uint8_t r = op_tsb(t);
    memory_write(addr, r);
}

#define _opcode_rmb(BIT)            \
_opcode(rmb##BIT){                  \
    uint16_t addr = zero_page();    \
    AVR_RAM[addr] &= ~(1 << BIT);   \
}

#define _opcode_smb(BIT)            \
_opcode(smb##BIT){                  \
    uint16_t addr = zero_page();    \
    AVR_RAM[addr] |= 1 << BIT;      \
}

_opcode_rmb(0);
_opcode_rmb(1);
_opcode_rmb(2);
_opcode_rmb(3);
_opcode_rmb(4);
_opcode_rmb(5);
_opcode_rmb(6);
_opcode_rmb(7);
_opcode_smb(0);
_opcode_smb(1);
_opcode_smb(2);
_opcode_smb(3);
_opcode_smb(4);
_opcode_smb(5);
_opcode_smb(6);
_opcode_smb(7);

// #endregion

// #region COMPARES

// #region CMP
static void op_cmp(uint8_t v) {
    asm volatile (
        "cp   %[a],%[v]"
        :
        : [a]"l"(a),
          [v]"r"(v)
    );
    _update_flags_NZC();
}

_opcode(cmp_imm) {
    uint8_t t = immediate();
    op_cmp(t);
}

_opcode(cmp_ind) {
    uint16_t addr = indirect_zero_page();
    uint8_t t = memory_read(addr);
    op_cmp(t);
}

_opcode(cmp_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    op_cmp(t);
}

_opcode(cmp_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    uint8_t t = AVR_RAM[addr];
    op_cmp(t);
}

_opcode(cmp_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    op_cmp(t);
}

_opcode(cmp_abs_x) {
    uint16_t addr = absolute_indexed_x();
    uint8_t t = memory_read(addr);
    op_cmp(t);
}

_opcode(cmp_abs_y) {
    uint16_t addr = absolute_indexed_y();
    uint8_t t = memory_read(addr);
    op_cmp(t);
}

_opcode(cmp_x_ind) {
    uint16_t addr = indexed_indirect();
    uint8_t t = memory_read(addr);
    op_cmp(t);
}

_opcode(cmp_ind_y) {
    uint16_t addr = indirect_indexed();
    uint8_t t = memory_read(addr);
    op_cmp(t);
}

// #endregion

// #region CPX
static void op_cpx(uint8_t v) {
    asm volatile (
        "cp   %[x],%[v]"
        :
        : [x]"l"(x),
          [v]"r"(v)
    );
    _update_flags_NZC();
}

_opcode(cpx_imm) {
    uint8_t t = immediate();
    op_cpx(t);
}

_opcode(cpx_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = AVR_RAM[addr];
    op_cpx(t);
}

_opcode(cpx_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    op_cpx(t);
}

// #endregion

// #region CPY
static void op_cpy(uint8_t v) {
    asm volatile (
        "cp   %[y],%[v]"
        :
        : [y]"l"(y),
          [v]"r"(v)
    );
    _update_flags_NZC();
}

_opcode(cpy_imm) {
    uint8_t t = immediate();
    op_cpy(t);
}

_opcode(cpy_zpg) {
    uint16_t addr = zero_page();
    uint8_t t = memory_read(addr);
    op_cpy(t);
}

_opcode(cpy_abs) {
    uint16_t addr = absolute();
    uint8_t t = memory_read(addr);
    op_cpy(t);
}

// #endregion

// #endregion

// #region LOAD/STORE

// #region LDA

_opcode(lda_imm) {
    a = immediate();
    _update_flags_LD(a);
}

_opcode(lda_ind) {
    uint16_t addr = indirect_zero_page();
    a = memory_read(addr);
    _update_flags_LD(a);
}

_opcode(lda_zpg) {
    uint16_t addr = zero_page();
    a = AVR_RAM[addr];
    _update_flags_LD(a);
}

_opcode(lda_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    a = AVR_RAM[addr];
    _update_flags_LD(a);
}

_opcode(lda_abs) {
    uint16_t addr = absolute();
    a = memory_read(addr);
    _update_flags_LD(a);
}

_opcode(lda_abs_x) {
    uint16_t addr = absolute_indexed_x();
    a = memory_read(addr);
    _update_flags_LD(a);
}

_opcode(lda_abs_y) {
    uint16_t addr = absolute_indexed_y();
    a = memory_read(addr);
    _update_flags_LD(a);
}

_opcode(lda_x_ind) {
    uint16_t addr = indexed_indirect();
    a = memory_read(addr);
    _update_flags_LD(a);
}

_opcode(lda_ind_y) {
    uint16_t addr = indirect_indexed();
    a = memory_read(addr);
    _update_flags_LD(a);
}

// #endregion

// #region STA

_opcode(sta_ind) {
    uint16_t addr = indirect_zero_page();
    AVR_RAM[addr] = a;
}

_opcode(sta_zpg) {
    uint16_t addr = zero_page();
    AVR_RAM[addr] = a;
}

_opcode(sta_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    AVR_RAM[addr] = a;
}

_opcode(sta_abs) {
    uint16_t addr = absolute();
    memory_write(addr, a);
}

_opcode(sta_abs_x) {
    uint16_t addr = absolute_indexed_x();
    memory_write(addr, a);
}

_opcode(sta_abs_y) {
    uint16_t addr = absolute_indexed_y();
    memory_write(addr, a);
}

_opcode(sta_x_ind) {
    uint16_t addr = indexed_indirect();
    memory_write(addr, a);
}

_opcode(sta_ind_y) {
    uint16_t addr = indirect_indexed();
    memory_write(addr, a);
}

// #endregion

// #region LDX

_opcode(ldx_imm) {
    x = immediate();
    _update_flags_LD(x);
}

_opcode(ldx_zpg) {
    uint16_t addr = zero_page();
    x = AVR_RAM[addr];
    _update_flags_LD(x);
}

_opcode(ldx_zpg_y) {
    uint16_t addr = zero_page_indexed_y();
    x = AVR_RAM[addr];
    _update_flags_LD(x);
}

_opcode(ldx_abs) {
    uint16_t addr = absolute();
    x = memory_read(addr);
    _update_flags_LD(x);
}

_opcode(ldx_abs_y) {
    uint16_t addr = absolute_indexed_y();
    x = memory_read(addr);
    _update_flags_LD(x);
}

// #endregion

// #region STX

_opcode(stx_zpg) {
    uint16_t addr = zero_page();
    AVR_RAM[addr] = x;
}

_opcode(stx_zpg_y) {
    uint16_t addr = zero_page_indexed_y();
    AVR_RAM[addr] = x;
}

_opcode(stx_abs) {
    uint16_t addr = absolute();
    memory_write(addr, x);
}

// #endregion

// #region LDY

_opcode(ldy_imm) {
    y = immediate();
    _update_flags_LD(y);
}

_opcode(ldy_zpg) {
    uint16_t addr = zero_page();
    y = AVR_RAM[addr];
    _update_flags_LD(y);
}

_opcode(ldy_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    y = AVR_RAM[addr];
    _update_flags_LD(y);
}

_opcode(ldy_abs) {
    uint16_t addr = absolute();
    y = memory_read(addr);
    _update_flags_LD(y);
}

_opcode(ldy_abs_x) {
    uint16_t addr = absolute_indexed_x();
    y = memory_read(addr);
    _update_flags_LD(y);
}

// #endregion

// #region STY

_opcode(sty_zpg) {
    uint16_t addr = zero_page();
    AVR_RAM[addr] = y;
}

_opcode(sty_zpg_x) {
    uint16_t addr = zero_page_indexed_y();
    AVR_RAM[addr] = y;
}

_opcode(sty_abs) {
    uint16_t addr = absolute();
    memory_write(addr, y);
}

// #endregion

// #region STZ

_opcode(stz_zpg) {
    uint16_t addr = zero_page();
    AVR_RAM[addr] = 0;
}

_opcode(stz_zpg_x) {
    uint16_t addr = zero_page_indexed_x();
    AVR_RAM[addr] = 0;
}

_opcode(stz_abs) {
    uint16_t addr = absolute();
    memory_write(addr, 0);
}

_opcode(stz_abs_x) {
    uint16_t addr = absolute_indexed_x();
    memory_write(addr, 0);
}

// #endregion

// #endregion

// #region TRANSFERS

_opcode(txa) {
    a = x;
    _update_flags_LD(a);
}

_opcode(tax) {
    x = a;
    _update_flags_LD(x);
}

_opcode(txs) {
    s = x;
    _update_flags_LD(s);
}

_opcode(tsx) {
    x = s;
    _update_flags_LD(x);
}

_opcode(tya) {
    a = y;
    _update_flags_LD(a);
}

_opcode(tay) {
    y = a;
    _update_flags_LD(y);
}

// #endregion

// #region PUSH/POP

_opcode(php) {
    push(p | 0x30);
}

_opcode(plp) {
    p = pop();
}

_opcode(pha) {
    push(a);
}

_opcode(pla) {
    a = pop();
}

_opcode(phy) {
    push(y);
}

_opcode(ply) {
    y = pop();
}

_opcode(phx) {
    push(x);
}

_opcode(plx) {
    x = pop();
}

// #endregion

// #region JUMPS

_opcode(bpl) {
    int8_t o = (int8_t) immediate();
    if (!FLAG_IS_SET(N)) {
        INC_PC(o);
    }
}

_opcode(bmi) {
    int8_t o = (int8_t) immediate();
    if (FLAG_IS_SET(N)) {
        INC_PC(o);
    }
}

_opcode(bvc) {
    int8_t o = (int8_t) immediate();
    if (!FLAG_IS_SET(V)) {
        INC_PC(o);
    }
}

_opcode(bvs) {
    int8_t o = (int8_t) immediate();
    if (FLAG_IS_SET(V)) {
        INC_PC(o);
    }
}

_opcode(bcc) {
    int8_t o = (int8_t) immediate();
    if (!FLAG_IS_SET(C)) {
        INC_PC(o);
    }
}

_opcode(bcs) {
    int8_t o = (int8_t) immediate();
    if (FLAG_IS_SET(C)) {
        INC_PC(o);
    }
}

_opcode(bne) {
    int8_t o = (int8_t) immediate();
    if (!FLAG_IS_SET(Z)) {
        INC_PC(o);
    }
}

_opcode(beq) {
    int8_t o = (int8_t) immediate();
    if (FLAG_IS_SET(Z)) {
        INC_PC(o);
    }
}

_opcode(bra) {
    int8_t o = (int8_t) immediate();
    INC_PC(o);
}

_opcode(jmp_abs) {
    uint16_t addr = absolute();
    SET_PC(addr);
}

_opcode(jmp_ind) {
    uint16_t addr = absolute_indirect();
    SET_PC(addr);
}

_opcode(jmp_x_ind) {
    uint16_t addr = absolute_indexed_indirect();
    SET_PC(addr);
}

_opcode(jsr) {
    uint16_t addr = absolute();
    push(high(pc));
    push(low(pc));
    SET_PC(addr);
}

_opcode(rts) {
    uint8_t ipl = pop();
    uint8_t iph = pop();
    pc = word(iph, ipl);
}

_opcode(rti) {
    p = pop();
    uint8_t ipl = pop();
    uint8_t iph = pop();
    pc = word(iph, ipl);
}

#define _opcode_bbr(BIT) _opcode_bbt(bbr##BIT, BIT, !)
#define _opcode_bbs(BIT) _opcode_bbt(bbs##BIT, BIT,  )
#define _opcode_bbt(NAME, BIT, S)       \
_opcode(NAME) {                         \
    uint16_t addr = zero_page();        \
    uint8_t t = AVR_RAM[addr];          \
    int8_t o = (int8_t) immediate();    \
    if (S (t & (1 << BIT))) {           \
        INC_PC(o);                      \
    }                                   \
}

_opcode_bbr(0)
_opcode_bbr(1)
_opcode_bbr(2)
_opcode_bbr(3)
_opcode_bbr(4)
_opcode_bbr(5)
_opcode_bbr(6)
_opcode_bbr(7)
_opcode_bbs(0)
_opcode_bbs(1)
_opcode_bbs(2)
_opcode_bbs(3)
_opcode_bbs(4)
_opcode_bbs(5)
_opcode_bbs(6)
_opcode_bbs(7)

// #endregion

// #region FLAGS

_opcode(clc) {
    CLR_FLAG(C);
}

_opcode(sec) {
    SET_FLAG(C);
}

_opcode(cli_) {
    CLR_FLAG(I);
}

_opcode(sei_) {
    SET_FLAG(I);
}

_opcode(clv) {
    CLR_FLAG(V);
}

_opcode(cld) {
    CLR_FLAG(D);
}

_opcode(sed) {
    SET_FLAG(D);
}

// #endregion

// #region STP/WAI

_opcode(stp) {
    cli();
    while (1) {
        sleep_mode();
    }
}

_opcode(wai) {
    sleep_enable();
    sleep_cpu();
    sleep_disable();
}

// #endregion

// #endregion

const PROGMEM opcode_fn opcode_table[OPCODE_TABLE_SIZE] = {
//   -0         -1         -2         -3         -4         -5         -6         -7         -8         -9         -a         -b         -c         -d         -e         -f
    brk,       ora_x_ind, undefined, undefined, tsb_zpg,   ora_zpg,   asl_zpg,   rmb0,      php,       ora_imm,   asl_a,     undefined, tsb_abs,   ora_abs,   asl_abs,   bbr0, // 0-
    bpl,       ora_ind_y, ora_ind,   undefined, trb_zpg,   ora_zpg_x, asl_zpg_x, rmb1,      clc,       ora_abs_y, inc_a,     undefined, trb_abs,   ora_abs_x, asl_abs_x, bbr1, // 1-
    jsr,       and_x_ind, undefined, undefined, bit_zpg,   and_zpg,   rol_zpg,   rmb2,      plp,       and_imm,   rol_a,     undefined, bit_abs,   and_abs,   rol_abs,   bbr2, // 2-
    bmi,       and_ind_y, and_ind,   undefined, bit_zpg_x, and_zpg_x, rol_zpg_x, rmb3,      sec,       and_abs_y, dec_a,     undefined, bit_abs_x, and_abs_x, rol_abs_x, bbr3, // 3-
    rti,       eor_x_ind, undefined, undefined, undefined, eor_zpg,   lsr_zpg,   rmb4,      pha,       eor_imm,   lsr_a,     undefined, jmp_abs,   eor_abs,   lsr_abs,   bbr4, // 4-
    bvc,       eor_ind_y, eor_ind,   undefined, undefined, eor_zpg_x, lsr_zpg_x, rmb5,      cli_,      eor_abs_y, phy,       undefined, undefined, eor_abs_x, lsr_abs_x, bbr5, // 5-
    rts,       adc_x_ind, undefined, undefined, stz_zpg,   adc_zpg,   ror_zpg,   rmb6,      pla,       adc_imm,   ror_a,     undefined, jmp_ind,   adc_abs,   ror_abs,   bbr6, // 6-
    bvs,       adc_ind_y, adc_ind,   undefined, stz_zpg_x, adc_zpg_x, ror_zpg_x, rmb7,      sei_,      adc_abs_y, ply,       undefined, jmp_x_ind, adc_abs_x, ror_abs_x, bbr7, // 7-
    bra,       sta_x_ind, undefined, undefined, sty_zpg,   sta_zpg,   stx_zpg,   smb0,      dey,       bit_imm,   txa,       undefined, sty_abs,   sta_abs,   stx_abs,   bbs0, // 8-
    bcc,       sta_ind_y, sta_ind,   undefined, sty_zpg_x, sta_zpg_x, stx_zpg_y, smb1,      tya,       sta_abs_y, txs,       undefined, stz_abs,   sta_abs_x, stz_abs_x, bbs1, // 9-
    ldy_imm,   lda_x_ind, ldx_imm,   undefined, ldy_zpg,   lda_zpg,   ldx_zpg,   smb2,      tay,       lda_imm,   tax,       undefined, ldy_abs,   lda_abs,   ldx_abs,   bbs2, // a-
    bcs,       lda_ind_y, lda_ind,   undefined, ldy_zpg_x, lda_zpg_x, ldx_zpg_y, smb3,      clv,       lda_abs_y, tsx,       undefined, ldy_abs_x, lda_abs_x, ldx_abs_y, bbs3, // b-
    cpy_imm,   cmp_x_ind, undefined, undefined, cpy_zpg,   cmp_zpg,   dec_zpg,   smb4,      iny,       cmp_imm,   dex,       wai,       cpy_abs,   cmp_abs,   dec_abs,   bbs4, // c-
    bne,       cmp_ind_y, cmp_ind,   undefined, undefined, cmp_zpg_x, dec_zpg_x, smb5,      cld,       cmp_abs_y, phx,       stp,       undefined, cmp_abs_x, dec_abs_x, bbs5, // d-
    cpx_imm,   sbc_x_ind, undefined, undefined, cpx_zpg,   sbc_zpg,   inc_zpg,   smb6,      inx,       sbc_imm,   nop,       undefined, cpx_abs,   sbc_abs,   inc_abs,   bbs6, // e-
    beq,       sbc_ind_y, sbc_ind,   undefined, undefined, sbc_zpg_x, inc_zpg_x, smb7,      sed,       sbc_abs_y, plx,       undefined, undefined, sbc_abs_x, inc_abs_x, bbs7, // f-
};
