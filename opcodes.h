#pragma once

enum OpCode {
    NOP       = 0xEA,

    ADC_IMM   = 0x69,
    ADC_ZPG   = 0x65,
    ADC_ZPG_X = 0x75,
    ADC_ABS   = 0x6D,
    ADC_ABS_X = 0x7D,
    ADC_ABS_Y = 0x79,
    ADC_IND_X = 0x61,
    ADC_IND_Y = 0x71,

    SBC_IMM   = 0xE9,
    SBC_ZPG   = 0xE5,
    SBC_ZPG_X = 0xF5,
    SBC_ABS   = 0xED,
    SBC_ABS_X = 0xFD,
    SBC_ABS_Y = 0xF9,
    SBC_IND_X = 0xE1,
    SBC_IND_Y = 0xF1,

    AND_IMM   = 0x29,
    AND_ZPG   = 0x25,
    AND_ZPG_X = 0x35,
    AND_ABS   = 0x2D,
    AND_ABS_X = 0x3D,
    AND_ABS_Y = 0x39,
    AND_IND_X = 0x21,
    AND_IND_Y = 0x31,

    ORA_IMM   = 0x09,
    ORA_ZPG   = 0x05,
    ORA_ZPG_X = 0x15,
    ORA_ABS   = 0x0D,
    ORA_ABS_X = 0x1D,
    ORA_ABS_Y = 0x19,
    ORA_IND_X = 0x01,
    ORA_IND_Y = 0x11,

    EOR_IMM   = 0x49,
    EOR_ZPG   = 0x45,
    EOR_ZPG_X = 0x55,
    EOR_ABS   = 0x4D,
    EOR_ABS_X = 0x5D,
    EOR_ABS_Y = 0x59,
    EOR_IND_X = 0x41,
    EOR_IND_Y = 0x51,

    ASL_A     = 0x0A,
    ASL_ZPG   = 0x06,
    ASL_ZPG_X = 0x16,
    ASL_ABS   = 0x0E,
    ASL_ABS_X = 0x1E,

    LSR_A     = 0x4A,
    LSR_ZPG   = 0x46,
    LSR_ZPG_X = 0x56,
    LSR_ABS   = 0x4E,
    LSR_ABS_X = 0x5E,

    ROL_A     = 0x2A,
    ROL_ZPG   = 0x26,
    ROL_ZPG_X = 0x36,
    ROL_ABS   = 0x2E,
    ROL_ABS_X = 0x3E,

    ROR_A     = 0x6A,
    ROR_ZPG   = 0x66,
    ROR_ZPG_X = 0x76,
    ROR_ABS   = 0x6E,
    ROR_ABS_X = 0x7E,

    BIT_ZPG   = 0x24,
    BIT_ABS   = 0x2C,

    BPL       = 0x10,
    BMI       = 0x30,
    BVC       = 0x50,
    BVS       = 0x70,
    BCC       = 0x90,
    BCS       = 0xB0,
    BNE       = 0xD0,
    BEQ       = 0xF0,

    JMP_ABS   = 0x4C,
    JMP_IND   = 0x6C,

    JSR       = 0x20,
    RTS       = 0x60,
    RTI       = 0x40,

    BRK       = 0x00,

    CMP_IMM   = 0xC9,
    CMP_ZPG   = 0xC5,
    CMP_ZPG_X = 0xD5,
    CMP_ABS   = 0xCD,
    CMP_ABS_X = 0xDD,
    CMP_ABS_Y = 0xD9,
    CMP_IND_X = 0xC1,
    CMP_IND_Y = 0xD1,

    CPX_IMM   = 0xE0,
    CPX_ZPG   = 0xE4,
    CPX_ABS   = 0xEC,

    CPY_IMM   = 0xC0,
    CPY_ZPG   = 0xC4,
    CPY_ABS   = 0xCC,

    DEC_ZPG   = 0xC6,
    DEC_ZPG_X = 0xD6,
    DEC_ABS   = 0xCE,
    DEC_ABS_X = 0xDE,

    INC_ZPG   = 0xE6,
    INC_ZPG_X = 0xF6,
    INC_ABS   = 0xEE,
    INC_ABS_X = 0xFE,

    CLC       = 0x18,
    SEC       = 0x38,
    CLI       = 0x58,
    SEI       = 0x78,
    CLV       = 0xB8,
    CLD       = 0xD8,
    SED       = 0xF8,

    LDA_IMM   = 0xA9,
    LDA_ZPG   = 0xA5,
    LDA_ZPG_X = 0xB5,
    LDA_ABS   = 0xAD,
    LDA_ABS_X = 0xBD,
    LDA_ABS_Y = 0xB9,
    LDA_IND_X = 0xA1,
    LDA_IND_Y = 0xB1,

    LDX_IMM   = 0xA2,
    LDX_ZPG   = 0xA6,
    LDX_ZPG_Y = 0xB6,
    LDX_ABS   = 0xAE,
    LDX_ABS_Y = 0xBE,

    LDY_IMM   = 0xA0,
    LDY_ZPG   = 0xA4,
    LDY_ZPG_X = 0xB4,
    LDY_ABS   = 0xAC,
    LDY_ABS_X = 0xBC,

    STA_ZPG   = 0x85,
    STA_ZPG_X = 0x95,
    STA_ABS   = 0x8D,
    STA_ABS_X = 0x9D,
    STA_ABS_Y = 0x99,
    STA_IND_X = 0x81,
    STA_IND_Y = 0x91,

    STX_ZPG   = 0x86,
    STX_ZPG_Y = 0x96,
    STX_ABS   = 0x8E,

    STY_ZPG   = 0x84,
    STY_ZPG_X = 0x94,
    STY_ABS   = 0x8C,

    TAX       = 0xAA,
    TXA       = 0x8A,
    DEX       = 0xCA,
    INX       = 0xE8,

    TAY       = 0xA8,
    TYA       = 0x98,
    DEY       = 0x88,
    INY       = 0xC8,

    TXS       = 0x9A,
    TSX       = 0xBA,
    PHA       = 0x48,
    PLA       = 0x68,
    PHP       = 0x08,
    PLP       = 0x28,

    // illegal instructions

    // weird nops
    NOP_1A    = 0x1A,  // implied      1    2
    NOP_3A    = 0x3A,  // implied      1    2
    NOP_5A    = 0x5A,  // implied      1    2
    NOP_7A    = 0x7A,  // implied      1    2
    NOP_DA    = 0xDA,  // implied      1    2
    NOP_FA    = 0xFA,  // implied      1    2
    NOP_80    = 0x80,  // immediate    2    2
    NOP_82    = 0x82,  // immediate    2    2
    NOP_89    = 0x89,  // immediate    2    2
    NOP_C2    = 0xC2,  // immediate    2    2
    NOP_E2    = 0xE2,  // immediate    2    2
    NOP_04    = 0x04,  // zeropage     2    3
    NOP_44    = 0x44,  // zeropage     2    3
    NOP_64    = 0x64,  // zeropage     2    3
    NOP_14    = 0x14,  // zeropage,X   2    4
    NOP_34    = 0x34,  // zeropage,X   2    4
    NOP_54    = 0x54,  // zeropage,X   2    4
    NOP_74    = 0x74,  // zeropage,X   2    4
    NOP_D4    = 0xD4,  // zeropage,X   2    4
    NOP_F4    = 0xF4,  // zeropage,X   2    4
    NOP_0C    = 0x0C,  // absolute     3    4
    NOP_1C    = 0x1C,  // absolute,X   3    4*
    NOP_3C    = 0x3C,  // absolute,X   3    4*
    NOP_5C    = 0x5C,  // absolute,X   3    4*
    NOP_7C    = 0x7C,  // absolute,X   3    4*
    NOP_DC    = 0xDC,  // absolute,X   3    4*
    NOP_FC    = 0xFC,  // absolute,X   3    4*

    // JAM opcodes
    JAM_02    = 0x02,
    JAM_12    = 0x12,
    JAM_22    = 0x22,
    JAM_32    = 0x32,
    JAM_42    = 0x42,
    JAM_52    = 0x52,
    JAM_62    = 0x62,
    JAM_72    = 0x72,
    JAM_92    = 0x92,
    JAM_B2    = 0xB2,
    JAM_D2    = 0xD2,
    JAM_F2    = 0xF2,
};
