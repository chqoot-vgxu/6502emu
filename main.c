#include <stdint.h>
#include <stddef.h>

#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define ram ((uint8_t*)0)
#define stack ((uint8_t*)0x1FF);


const PROGMEM uint8_t rom[] = {
// reset:   .word $0503
    0x03, 0x05,
// irq:     .word $0503
    0x03, 0x05,

// main:
    0xA9, 0x10,         // lda #$10
    0xA2, 0x00,         // ldx #$00  ; baud 115200
    0x20, 0x41, 0x05,   // jsr uart_init
    0xA9, 0x20,         // lda #$20  ; (1 << PB5)
    0x85, 0x24,         // sta $24   ; DDRB
// loop:
    0xA9, 0x20,         // lda #$20
    0x85, 0x23,         // sta $23   ; PINB
    0xA0, 0x00,         // ldy #$00
// tx_next:
    0xB9, 0x5E, 0x05,   // lda hello_world,y
    0x20, 0x52, 0x05,   // jsr uart_tx
    0xC8,               // iny
    0xC0, 0x0E,         // cpy #$14
    0xD0, 0xF5,         // bne tx_next
    0xA9, 0xFF,         // lda #$FF
    0x8D, 0x50, 0x02,   // sta $0250 ; delay input low
    0x8D, 0x51, 0x02,   // sta $0251 ; delay input high
    0x20, 0x2D, 0x05,   // jsr delay
    0xB8,               // clv
    0x50, 0xE1,         // bvc loop

// delay
    0xAD, 0x50, 0x02,   // lda $0250
    0xD0, 0x08,         // bne label
    0xAD, 0x51, 0x02,   // lda $0251
    0xF0, 0x09,         // beq zero
    0xCE, 0x51, 0x02,   // dec $0251
// label
    0xCE, 0x50, 0x02,   // dec $0250
    0xB8,               // clv
    0x50, 0xED,         // bvc delay
// zero
    0x60,               // rts

// uart_init:
    0x85, 0xC4,         // sta $C4  ; UBRR0L
    0x86, 0xC5,         // stx $C5  ; UBRR0H
    0xA9, 0x02,         // lda #$02 ; (1 << U2X0)
    0x85, 0xC0,         // sta $C0  ; UCSR0A
    0xA9, 0x08,         // lda #$08 ; (1 << TXEN0)
    0x85, 0xC1,         // sta $C1  ; UCSR0B
    0xA9, 0x0E,         // lda #$0E ; (1 << USBS0) | (3 << UCSZ00)
    0x85, 0xC2,         // sta $C2  ; UCSR0C
    0x60,               // rts

// uart_tx:
    0x48,               // pha
// wait
    0xA9, 0x20,         // lda #$20 ; (1 << UDRE0)
    0x24, 0xC0,         // bit $C0  ; UCSR0A
    0xF0, 0xFA,         // beq wait
    0x68,               // pla
    0x85, 0xC6,         // sta $C6  ; UDR0
    0x60,               // rts

// hello_world: .string "Hello, world!\n"
    'H', 'e', 'l', 'l', 'o', ',', ' ', 'w', 'o', 'r', 'l', 'd', '!', '\n', '\0'
};


// // BLINK
// const PROGMEM uint8_t rom[] = {
// // reset
//     0x03, 0x05,
// // irq
//     0x03, 0x05,
// // main
//     0xA9, 0x20,         // lda #$20
//     0x85, 0x24,         // sta $24
// // loop
//     0x85, 0x23,         // sta $23
//     0xA9, 0xFF,         // lda #$FF
//     0x8D, 0x50, 0x02,   // sta $0250
//     0x8D, 0x51, 0x02,   // sta $0251
//     0x20, 0x19, 0x05,   // jsr delay
//     0xA9, 0x20,         // lda #$20
//     0xB8,               // clv
//     0x50, 0xEE,         // bvc delay

// // delay
//     0xAD, 0x50, 0x02,   // lda $0250
//     0xD0, 0x08,         // bne label
//     0xAD, 0x51, 0x02,   // lda $0251
//     0xF0, 0x09,         // beq zero
//     0xCE, 0x51, 0x02,   // dec $0251
// // label
//     0xCE, 0x50, 0x02,   // dec $0250
//     0xB8,               // clv
//     0x50, 0xED,         // bvc delay
// // zero
//     0x60,               // rts
// };


register uint8_t sp  asm("r9");
register size_t  ip  asm("r10");
register uint8_t ipl asm("r10");
register uint8_t iph asm("r11");
register uint8_t a   asm("r12");
register uint8_t f   asm("r13");
register uint8_t x   asm("r15");
register uint8_t y   asm("r14");


#define WORD(h, l) (uint16_t)(((h) << 8) | (l))
#define SET_IP(n) ip  = (n)
#define INC_IP(n) ip += (n)
#define UPDATE_FLAG_REGISTER() f = SREG
#define SET_FLAG(n)  f |=  (1 << (n))
#define CLR_FLAG(n)  f &= ~(1 << (n))
#define READ_FLAG(n) (f & (1 << (n)))
#define STACK(sp) ram[0x100 + (sp)]
#define RESET_VEC (0x0500u)
#define IRQ_VEC   (0x0502u)

#define ADC_(t)    asm("adc %0, %1" : "+r"(a) : "r"((t)) : "cc")
#define SBC_(t)    asm("sbc %0, %1" : "+r"(a) : "r"((t)) : "cc")
#define ROL_(t)    asm("rol %0" : "+r"((t)) :: "cc")
#define ROR_(t)    asm("ror %0" : "+r"((t)) :: "cc")
#define CMP_(a, b) asm("cp %0, %1" :: "r" ((a)), "r" ((b)) : "cc")
#define BIT_(t)    asm("mov __tmp_reg__, %0"       "\n\t"   \
                       "and __tmp_reg__, %1"       "\n\t"   \
                       :: "r"(t), "r"(a) : "cc")

uint8_t memory_read(uint16_t addr) {
    return (addr < 0x0500u ? ram[addr] : pgm_read_byte(rom + (addr - 0x0500u)));
}

void memory_write(uint16_t addr, uint8_t v) {
    if (addr < 0x0500u) {
        ram[addr] = v;
    }
}

void push(uint8_t reg) {
    STACK(sp) = reg;
    sp--;
}

uint8_t pop() {
    sp++;
    return STACK(sp);
}

void set_flags_on_load(int8_t reg) {
    if (reg < 0) SET_FLAG(SREG_N);
    else CLR_FLAG(SREG_N);
    if (reg == 0) SET_FLAG(SREG_Z);
    else CLR_FLAG(SREG_Z);
}

void set_flags_on_bit(int8_t t) {
    UPDATE_FLAG_REGISTER();
    if (t & (1 << 6)) SET_FLAG(SREG_V);
    else CLR_FLAG(SREG_V);
}

uint8_t immediate() {
    INC_IP(1);
    return memory_read(ip);
}

uint16_t zero_page() {
    return immediate();
}

uint16_t zero_page_indexed(uint8_t reg) {
    return immediate() + reg;
}

uint16_t absolute() {
    uint8_t l = immediate();
    uint8_t h = immediate();
    return WORD(h, l);
}

uint16_t absolute_indexed(uint8_t reg) {
    return absolute() + reg;
}

uint16_t indexed_indirect() {
    uint8_t z = immediate() + x;
    uint16_t i = WORD(ram[z+1], ram[z]);
    return WORD(memory_read(i+1), memory_read(i));
}

uint16_t indirect_indexed() {
    uint8_t z = immediate();
    uint8_t l = ram[z];
    uint8_t h = ram[z+1];
    return memory_read(WORD(h, l) + y);
}


int main() {
    ipl = memory_read(RESET_VEC);
    iph = memory_read(RESET_VEC + 1);
    sp = 0xFF;

    for (;;) {
        uint8_t instruction = immediate();
        switch (instruction) {
            case 0xEA: break; // NOP


            case 0x69: { // ADC IMM
                uint8_t t = immediate();
                ADC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x65: { // ADC ZPG
                uint16_t addr = zero_page();
                uint8_t t = ram[addr];
                ADC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x75: { // ADC ZPG,X
                uint16_t addr = zero_page_indexed(x);
                uint8_t t = ram[addr];
                ADC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x6D: { // ADC ABS
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                ADC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x7D: { // ADC ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = memory_read(addr);
                ADC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x79: { // ADC ABS,Y
                uint16_t addr = absolute_indexed(y);
                uint8_t t = memory_read(addr);
                ADC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x61: { // ADC (IMM,X)
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                ADC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x71: { // ADC (IMM),Y
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                ADC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0xE9: { // SBC IMM
                uint8_t t = immediate();
                SBC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xE5: { // SBC ZPG
                uint8_t t = ram[zero_page()];
                SBC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xF5: { // SBC ZPG,X
                uint8_t t = ram[zero_page_indexed(x)];
                SBC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xED: { // SBC ABS
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                SBC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xFD: { // SBC ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = memory_read(addr);
                SBC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xF9: { // SBC ABS,Y
                uint16_t addr = absolute_indexed(y);
                uint8_t t = memory_read(addr);
                SBC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xE1: { // SBC (IMM,X)
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                SBC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xF1: { // SBC (IMM),Y
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                SBC_(t);
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0x29: { // AND IMM
                a &= immediate();
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x25: { // AND ZPG
                a &= ram[zero_page()];
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x35: { // AND ZPG,X
                a &= ram[zero_page_indexed(x)];
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x2D: { // AND ABS
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                a &= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x3D: { // AND ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = memory_read(addr);
                a &= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x39: { // AND ABS,Y
                uint16_t addr = absolute_indexed(y);
                uint8_t t = memory_read(addr);
                a &= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x21: { // AND (IMM,X)
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                a &= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x31: { // AND (IMM),Y
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                a &= t;
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0x09: { // ORA IMM
                a |= immediate();
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x05: { // ORA ZPG
                a |= ram[zero_page()];
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x15: { // ORA ZPG,X
                a |= ram[zero_page_indexed(x)];
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x0D: { // ORA ABS
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                a |= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x1D: { // ORA ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = memory_read(addr);
                a |= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x19: { // ORA ABS,Y
                uint16_t addr = absolute_indexed(y);
                uint8_t t = memory_read(addr);
                a |= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x01: { // ORA (IMM,X)
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                a |= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x11: { // ORA (IMM),Y
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                a |= t;
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0x49: { // EOR IMM
                a ^= immediate();
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x45: { // EOR ZPG
                a ^= ram[zero_page()];
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x55: { // EOR ZPG,X
                a ^= ram[zero_page_indexed(x)];
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x4D: { // EOR ABS
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                a ^= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x5D: { // EOR ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = memory_read(addr);
                a ^= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x59: { // EOR ABS,Y
                uint16_t addr = absolute_indexed(y);
                uint8_t t = memory_read(addr);
                a ^= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x41: { // EOR (IMM,X)
                uint16_t addr = indexed_indirect();
                uint8_t t = memory_read(addr);
                a ^= t;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x51: { // EOR (IMM),Y
                uint16_t addr = indirect_indexed();
                uint8_t t = memory_read(addr);
                a ^= t;
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0x0A: { // ASL A
                a <<= 1;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x06: { // ASL ZPG
                ram[zero_page()] <<= 1;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x16: { // ASL ZPG,X
                ram[zero_page_indexed(x)] <<= 1;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x0E: { // ASL ABS
                ram[absolute()] <<= 1;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x1E: { // ASL ABS,X
                ram[absolute_indexed(x)] <<= 1;
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0x4A: { // LSR A
                a >>= 1;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x46: { // LSR ZPG
                ram[zero_page()] >>= 1;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x56: { // LSR ZPG,X
                ram[zero_page_indexed(x)] >>= 1;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x4E: { // LSR ABS
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                memory_write(addr, t >> 1);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x5E: { // LSR ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = memory_read(addr);
                memory_write(addr, t >> 1);
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0x2A: { // ROL A
                ROL_(a);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x26: { // ROL ZPG
                uint16_t addr = zero_page();
                uint8_t t = ram[addr];
                ROL_(t);
                UPDATE_FLAG_REGISTER();
                ram[addr] = t;
                break;
            }

            case 0x36: { // ROL ZPG,X
                uint16_t addr = zero_page_indexed(x);
                uint8_t t = ram[addr];
                ROL_(t);
                UPDATE_FLAG_REGISTER();
                ram[addr] = t;
                break;
            }

            case 0x2E: { // ROL ABS
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                ROL_(t);
                UPDATE_FLAG_REGISTER();
                memory_write(addr, t);
                break;
            }

            case 0x3E: { // ROL ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = memory_read(addr);
                ROL_(t);
                UPDATE_FLAG_REGISTER();
                memory_write(addr, t);
                break;
            }


            case 0x6A: { // ROR A
                ROR_(a);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0x66: { // ROR ZPG
                uint16_t addr = zero_page();
                uint8_t t = ram[addr];
                ROR_(t);
                UPDATE_FLAG_REGISTER();
                ram[addr] = t;
                break;
            }

            case 0x76: { // ROR ZPG,X
                uint16_t addr = zero_page_indexed(x);
                uint8_t t = ram[addr];
                ROR_(t);
                UPDATE_FLAG_REGISTER();
                ram[addr] = t;
                break;
            }

            case 0x6E: { // ROR ABS
                uint16_t addr = absolute();
                uint8_t t = ram[addr];
                ROR_(t);
                UPDATE_FLAG_REGISTER();
                memory_write(addr, t);
                break;
            }

            case 0x7E: { // ROR ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = ram[addr];
                ROR_(t);
                UPDATE_FLAG_REGISTER();
                memory_write(addr, t);
                break;
            }


            case 0x24: { // BIT ZPG
                uint8_t t = ram[zero_page()];
                BIT_(t);
                set_flags_on_bit(t);
                break;
            }

            case 0x2C: { // BIT ABS
                uint8_t t = memory_read(absolute());
                BIT_(t);
                set_flags_on_bit(t);
                break;
            }


            case 0x10: { // BPL
                int8_t o = immediate();
                if (!READ_FLAG(SREG_N)) {
                    INC_IP(o);
                }
                break;
            }

            case 0x30: { // BMI
                int8_t o = immediate();
                if (READ_FLAG(SREG_N)) {
                    INC_IP(o);
                }
                break;
            }

            case 0x50: { // BVC
                int8_t o = immediate();
                if (!READ_FLAG(SREG_V)) {
                    INC_IP(o);
                }
                break;
            }

            case 0x70: { // BVS
                int8_t o = immediate();
                if (READ_FLAG(SREG_V)) {
                    INC_IP(o);
                }
                break;
            }

            case 0x90: { // BCC
                int8_t o = immediate();
                if (!READ_FLAG(SREG_C)) {
                    INC_IP(o);
                }
                break;
            }

            case 0xB0: { // BCS
                int8_t o = immediate();
                if (READ_FLAG(SREG_C)) {
                    INC_IP(o);
                }
                break;
            }

            case 0xD0: { // BNE
                int8_t o = immediate();
                if (!READ_FLAG(SREG_Z)) {
                    INC_IP(o);
                }
                break;
            }

            case 0xF0: { // BEQ
                int8_t o = immediate();
                if (READ_FLAG(SREG_Z)) {
                    INC_IP(o);
                }
                break;
            }


            case 0x4C: { // JMP ABS
                uint16_t d = absolute();
                ip = d;
                break;
            }

            case 0x6C: { // JMP IND
                uint16_t addr = absolute();
                ipl = memory_read(addr);
                iph = memory_read(addr + 1);
                break;
            }


            case 0x20: { // JSR
                uint16_t d = absolute();
                push(iph);
                push(ipl);
                ip = d;
                break;
            }


            case 0x60: { // RTS
                ipl = pop();
                iph = pop();
                break;
            }

            case 0x40: { // RTI
                f = pop();
                ipl = pop();
                iph = pop();
                sei();
                break;
            }

            case 0x00: // BRK
                return 0x00;


            case 0xC9: { // CMP IMM
                uint8_t t = immediate();
                CMP_(a, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xC5: { // CMP ZPG
                uint8_t t = ram[zero_page()];
                CMP_(a, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xD5: { // CMP ZPG,X
                uint8_t t = ram[zero_page_indexed(x)];
                CMP_(a, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xCD: { // CMP ABS
                uint8_t t = memory_read(absolute());
                CMP_(a, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xDD: { // CMP ABS,X
                uint8_t t = memory_read(absolute_indexed(x));
                CMP_(a, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xD9: { // CMP ABS,Y
                uint8_t t = memory_read(absolute_indexed(y));
                CMP_(a, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xC1: { // CMP (IMM,X)
                uint8_t t = memory_read(indexed_indirect());
                CMP_(a, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xD1: { // CMP (IMM),Y
                uint8_t t = memory_read(indirect_indexed());
                CMP_(a, t);
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0xE0: { // CPX IMM
                uint8_t t = immediate();
                CMP_(x, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xE4: { // CPX ZPG
                uint8_t t = ram[zero_page()];
                CMP_(x, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xEC: { // CPX ABS
                uint8_t t = memory_read(absolute());
                CMP_(x, t);
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0xC0: { // CPY IMM
                uint8_t t = immediate();
                CMP_(y, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xC4: { // CPY ZPG
                uint8_t t = ram[zero_page()];
                CMP_(y, t);
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xCC: { // CPY ABS
                uint8_t t = memory_read(absolute());
                CMP_(y, t);
                UPDATE_FLAG_REGISTER();
                break;
            }


            case 0xC6: { // DEC ZPG
                ram[zero_page()]--;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xD6: { // DEC ZPG,X
                ram[zero_page_indexed(x)]--;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xCE: { // DEC ABS
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                t--;
                UPDATE_FLAG_REGISTER();
                memory_write(addr, t);
                break;
            }

            case 0xDE: { // DEC ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = memory_read(addr);
                t--;
                UPDATE_FLAG_REGISTER();
                memory_write(addr, t);
                break;
            }


            case 0xE6: { // INC ZPG
                ram[zero_page()]++;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xF6: { // INC ZPG,X
                ram[zero_page_indexed(x)]++;
                UPDATE_FLAG_REGISTER();
                break;
            }

            case 0xEE: { // INC ABS
                uint16_t addr = absolute();
                uint8_t t = memory_read(addr);
                t++;
                UPDATE_FLAG_REGISTER();
                memory_write(addr, t);
                break;
            }

            case 0xFE: { // INC ABS,X
                uint16_t addr = absolute_indexed(x);
                uint8_t t = memory_read(addr);
                t++;
                UPDATE_FLAG_REGISTER();
                memory_write(addr, t);
                break;
            }


            case 0x18: { // CLC
                CLR_FLAG(SREG_C);
                break;
            }


            case 0x38: { // SEC
                SET_FLAG(SREG_C);
                break;
            }


            case 0x58: { // CLI
                CLR_FLAG(SREG_I);
                break;
            }


            case 0x78: { // SEI
                SET_FLAG(SREG_I);
                break;
            }


            case 0xB8: { // CLV
                CLR_FLAG(SREG_V);
                break;
            }


            case 0xD8:  // CLD
                return 0xD8;


            case 0xF8: // SED
                return 0xF8;


            case 0xA9: { // LDA IMM
                a = immediate();
                set_flags_on_load(a);
                break;
            }

            case 0xA5: { // LDA ZPG
                a = ram[zero_page()];
                set_flags_on_load(a);
                break;
            }

            case 0xB5: { // LDA ZPG,X
                a = ram[zero_page_indexed(x)];
                set_flags_on_load(a);
                break;
            }

            case 0xAD: { // LDA ABS
                uint16_t addr = absolute();
                a = memory_read(addr);
                set_flags_on_load(a);
                break;
            }

            case 0xBD: { // LDA ABS,X
                uint16_t addr = absolute_indexed(x);
                a = memory_read(addr);
                set_flags_on_load(a);
                break;
            }

            case 0xB9: { // LDA ABS,Y
                uint16_t addr = absolute_indexed(y);
                a = memory_read(addr);
                set_flags_on_load(a);
                break;
            }

            case 0xA1: { // LDA (IMM,X)
                uint16_t addr = indexed_indirect();
                a = memory_read(addr);
                set_flags_on_load(a);
                break;
            }

            case 0xB1: { // LDA (IMM),Y
                uint16_t addr = indirect_indexed();
                a = memory_read(addr);
                set_flags_on_load(a);
                break;
            }


            case 0xA2: { // LDX IMM
                x = immediate();
                set_flags_on_load(x);
                break;
            }

            case 0xA6: { // LDX ZPG
                x = ram[zero_page()];
                set_flags_on_load(x);
                break;
            }

            case 0xB6: { // LDX ZPG,Y
                x = ram[zero_page_indexed(y)];
                set_flags_on_load(x);
                break;
            }

            case 0xAE: { // LDX ABS
                uint16_t addr = absolute();
                x = memory_read(addr);
                set_flags_on_load(x);
                break;
            }

            case 0xBE: { // LDX ABS,Y
                uint16_t addr = absolute_indexed(y);
                x = memory_read(addr);
                set_flags_on_load(x);
                break;
            }


            case 0xA0: { // LDY IMM
                y = immediate();
                set_flags_on_load(y);
                break;
            }

            case 0xA4: { // LDY ZPG
                y = ram[zero_page()];
                set_flags_on_load(y);
                break;
            }

            case 0xB4: { // LDY ZPG,X
                y = ram[zero_page_indexed(x)];
                set_flags_on_load(y);
                break;
            }

            case 0xAC: { // LDY ABS
                uint16_t addr = absolute();
                y = memory_read(addr);
                set_flags_on_load(y);
                break;
            }

            case 0xBC: { // LDY ABS,X
                uint16_t addr = absolute_indexed(x);
                y = memory_read(addr);
                set_flags_on_load(y);
                break;
            }


            case 0x85: { // STA ZPG
                ram[zero_page()] = a;
                break;
            }

            case 0x95: { // STA ZPG,X
                ram[zero_page_indexed(x)] = a;
                break;
            }

            case 0x8D: { // STA ABS
                memory_write(absolute(), a);
                break;
            }

            case 0x9D: { // STA ABS,X
                memory_write(absolute_indexed(x), a);
                break;
            }

            case 0x99: { // STA ABS,Y
                memory_write(absolute_indexed(y), a);
                break;
            }

            case 0x81: { // STA (IMM,X)
                memory_write(indexed_indirect(), a);
                break;
            }

            case 0x91: { // STA (IMM),Y
                memory_write(indirect_indexed(), a);
                break;
            }


            case 0x86: { // STX ZPG
                ram[zero_page()] = x;
                break;
            }

            case 0x96: { // STA ZPG,Y
                ram[zero_page_indexed(y)] = x;
                break;
            }

            case 0x8E: { // STX ABS
                memory_write(absolute(), x);
                break;
            }


            case 0x84: { // STY ZPG
                ram[zero_page()] = y;
                break;
            }

            case 0x94: { // STY ZPG,X
                ram[zero_page_indexed(x)] = y;
                break;
            }

            case 0x8C: { // STY ABS
                memory_write(absolute(), y);
                break;
            }


            case 0xAA: { // TAX
                x = a;
                break;
            }

            case 0x8A: { // TXA
                a = x;
                break;
            }

            case 0xCA: { // DEX
                x--;
                break;
            }

            case 0xE8: { // INX
                x++;
                break;
            }

            case 0xA8: { // TAY
                y = a;
                break;
            }

            case 0x98: { // TYA
                a = y;
                break;
            }

            case 0x88: { // DEY
                y--;
                break;
            }

            case 0xC8: { // INY
                y++;
                break;
            }


            case 0x9A: { // TXS
                sp = x;
                break;
            }

            case 0xBA: { // TSX
                x = sp;
                break;
            }

            case 0x48: { // PHA
                push(a);
                break;
            }

            case 0x68: { // PLA
                a = pop();
                break;
            }

            case 0x08: { // PHP
                push(f);
                break;
            }

            case 0x28: { // PLP
                f = pop();
                break;
            }


            default:
                return instruction;
        }
    }
}
