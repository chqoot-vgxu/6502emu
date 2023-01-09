
# region OPCODES

# region HELPERS

def read_uint16(rom: bytes, pc: int) -> int:
    low = rom[pc] & 0xff
    high = rom[pc+1] & 0xff
    return (high << 8) | low


def opc_imp(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+1], name)


def opc_acc(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+1], f"{name} a")


def opc_imm(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    imm = rom[pc+1]
    imm_char = f"'{imm:c}'" if imm >= ord(' ') else ''
    return (rom[pc:pc+2], f"{name} #${int(imm):02x} ; {imm} {imm_char}")


def opc_rel(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    rel = int(r) if (r := rom[pc+1]) < 127 else int(r) - 256
    return (rom[pc:pc+2], f"{name} .${int(rel):+}")


def opc_zpg(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+2], f"{name} ${int(rom[pc+1]):02x}")


def opc_zpg_x(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+2], f"{name} ${int(rom[pc+1]):02x},x")


def opc_zpg_y(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+2], f"{name} ${int(rom[pc+1]):02x},y")


def opc_abs(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+3], f"{name} ${int(read_uint16(rom, pc+1)):04x}")


def opc_abs_x(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+3], f"{name} ${int(read_uint16(rom, pc+1)):04x},x")


def opc_abs_y(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+3], f"{name} ${int(read_uint16(rom, pc+1)):04x},y")


def opc_ind(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+2], f"{name} (${int(read_uint16(rom, pc+1)):04x})")


def opc_x_ind(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+2], f"{name} (${int(read_uint16(rom, pc+1)):04x}),x")


def opc_ind_y(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+2], f"{name} (${int(read_uint16(rom, pc+1)):04x},y)")


def opc_abs_ind(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+3], f"{name} (${int(read_uint16(rom, pc+1)):04x})")


def opc_abs_x_ind(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+3], f"{name} (${int(read_uint16(rom, pc+1)):04x},x)")


def opc_zpg_rel(name: str, rom: bytes, pc: int) -> tuple[bytes, str]:
    return (rom[pc:pc+3], f"{name} ${int(rom[pc+1]):02x},${int(rom[pc+2]):02x}")

# endregion

def undefined(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("undefined", rom, pc)
def nop(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("nop", rom, pc)
def brk(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("brk", rom, pc)

# region ARITHMETIC

# region ADC
def adc_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("adc", rom, pc)
def adc_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("adc", rom, pc)
def adc_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("adc", rom, pc)
def adc_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("adc", rom, pc)
def adc_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("adc", rom, pc)
def adc_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("adc", rom, pc)
def adc_x_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_x_ind("adc", rom, pc)
def adc_ind_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind_y("adc", rom, pc)
def adc_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind("adc", rom, pc)
# endregion

# region SBC
def sbc_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("sbc", rom, pc)
def sbc_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("sbc", rom, pc)
def sbc_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("sbc", rom, pc)
def sbc_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("sbc", rom, pc)
def sbc_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("sbc", rom, pc)
def sbc_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("sbc", rom, pc)
def sbc_x_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_x_ind("sbc", rom, pc)
def sbc_ind_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind_y("sbc", rom, pc)
def sbc_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind("sbc", rom, pc)
# endregion

# region AND
def and_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("and", rom, pc)
def and_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("and", rom, pc)
def and_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("and", rom, pc)
def and_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("and", rom, pc)
def and_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("and", rom, pc)
def and_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("and", rom, pc)
def and_x_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_x_ind("and", rom, pc)
def and_ind_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind_y("and", rom, pc)
def and_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind("and", rom, pc)
# endregion

# region ORA
def ora_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("ora", rom, pc)
def ora_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("ora", rom, pc)
def ora_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("ora", rom, pc)
def ora_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("ora", rom, pc)
def ora_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("ora", rom, pc)
def ora_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("ora", rom, pc)
def ora_x_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_x_ind("ora", rom, pc)
def ora_ind_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind_y("ora", rom, pc)
def ora_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind("ora", rom, pc)
# endregion

# region EOR
def eor_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("eor", rom, pc)
def eor_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("eor", rom, pc)
def eor_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("eor", rom, pc)
def eor_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("eor", rom, pc)
def eor_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("eor", rom, pc)
def eor_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("eor", rom, pc)
def eor_x_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_x_ind("eor", rom, pc)
def eor_ind_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind_y("eor", rom, pc)
def eor_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind("eor", rom, pc)
# endregion

# region INC/DEC
def inc_a(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_acc("inc", rom, pc)
def inc_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("inc", rom, pc)
def inc_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("inc", rom, pc)
def inc_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("inc", rom, pc)
def inc_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("inc", rom, pc)
def dec_a(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_acc("dec", rom, pc)
def dec_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("dec", rom, pc)
def dec_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("dec", rom, pc)
def dec_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("dec", rom, pc)
def dec_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("dec", rom, pc)
def inx(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("inx", rom, pc)
def dex(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("dex", rom, pc)
def iny(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("iny", rom, pc)
def dey(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("dey", rom, pc)
# endregion

# endregion

# region COMPARES

# region CMP
def cmp_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("cmp", rom, pc)
def cmp_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("cmp", rom, pc)
def cmp_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("cmp", rom, pc)
def cmp_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("cmp", rom, pc)
def cmp_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("cmp", rom, pc)
def cmp_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("cmp", rom, pc)
def cmp_x_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_x_ind("cmp", rom, pc)
def cmp_ind_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind_y("cmp", rom, pc)
def cmp_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind("cmp", rom, pc)
# endregion

# region CPX
def cpx_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("cpx", rom, pc)
def cpx_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("cpx", rom, pc)
def cpx_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("cpx", rom, pc)

# endregion

# region CPY
def cpy_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("cpx", rom, pc)
def cpy_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("cpx", rom, pc)
def cpy_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("cpx", rom, pc)
# endregion

# endregion

# region BITS
def bit_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("bit", rom, pc)
def bit_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("bit", rom, pc)
def bit_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("bit", rom, pc)
def bit_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("bit", rom, pc)
def bit_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("bit", rom, pc)


def asl_a(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_acc("asl", rom, pc)
def asl_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("asl", rom, pc)
def asl_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("asl", rom, pc)
def asl_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("asl", rom, pc)
def asl_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("asl", rom, pc)


def lsr_a(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_acc("lsr", rom, pc)
def lsr_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("lsr", rom, pc)
def lsr_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("lsr", rom, pc)
def lsr_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("lsr", rom, pc)
def lsr_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("lsr", rom, pc)


def rol_a(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_acc("rol", rom, pc)
def rol_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("rol", rom, pc)
def rol_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("rol", rom, pc)
def rol_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("rol", rom, pc)
def rol_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("rol", rom, pc)


def ror_a(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_acc("ror", rom, pc)
def ror_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("ror", rom, pc)
def ror_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("ror", rom, pc)
def ror_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("ror", rom, pc)
def ror_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("ror", rom, pc)

def tsb_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("tsb", rom, pc)
def trb_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("trb", rom, pc)
def tsb_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("tsb", rom, pc)
def trb_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("trb", rom, pc)

def rmb0(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("rmb0", rom, pc)
def rmb1(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("rmb1", rom, pc)
def rmb2(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("rmb2", rom, pc)
def rmb3(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("rmb3", rom, pc)
def rmb4(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("rmb4", rom, pc)
def rmb5(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("rmb5", rom, pc)
def rmb6(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("rmb6", rom, pc)
def rmb7(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("rmb7", rom, pc)
def smb0(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("smb0", rom, pc)
def smb1(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("smb1", rom, pc)
def smb2(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("smb2", rom, pc)
def smb3(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("smb3", rom, pc)
def smb4(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("smb4", rom, pc)
def smb5(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("smb5", rom, pc)
def smb6(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("smb6", rom, pc)
def smb7(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("smb7", rom, pc)

# endregion

# region LOAD/STORE
def lda_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("lda", rom, pc)
def lda_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("lda", rom, pc)
def lda_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("lda", rom, pc)
def lda_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("lda", rom, pc)
def lda_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("lda", rom, pc)
def lda_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("lda", rom, pc)
def lda_x_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_x_ind("lda", rom, pc)
def lda_ind_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind_y("lda", rom, pc)
def lda_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind("lda", rom, pc)


def sta_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("sta", rom, pc)
def sta_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("sta", rom, pc)
def sta_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("sta", rom, pc)
def sta_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("sta", rom, pc)
def sta_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("sta", rom, pc)
def sta_x_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_x_ind("sta", rom, pc)
def sta_ind_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind_y("sta", rom, pc)
def sta_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_ind("sta", rom, pc)


def ldx_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("ldx", rom, pc)
def ldx_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("ldx", rom, pc)
def ldx_zpg_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_y("ldx", rom, pc)
def ldx_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("ldx", rom, pc)
def ldx_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("ldx", rom, pc)


def stx_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("stx", rom, pc)
def stx_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("stx", rom, pc)
def stx_zpg_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_y("stx", rom, pc)
def stx_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("stx", rom, pc)
def stx_abs_y(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_y("stx", rom, pc)


def ldy_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("ldy", rom, pc)
def ldy_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("ldy", rom, pc)
def ldy_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("ldy", rom, pc)
def ldy_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("ldy", rom, pc)
def ldy_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("ldy", rom, pc)


def sty_imm(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imm("sty", rom, pc)
def sty_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("sty", rom, pc)
def sty_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("sty", rom, pc)
def sty_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("sty", rom, pc)
def sty_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("sty", rom, pc)


def stz_zpg(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg("stz", rom, pc)
def stz_zpg_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_x("stz", rom, pc)
def stz_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("stz", rom, pc)
def stz_abs_x(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x("stz", rom, pc)
# endregion

# region STACK
def php(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("php", rom, pc)
def plp(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("plp", rom, pc)
def pha(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("pha", rom, pc)
def pla(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("pla", rom, pc)
def phx(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("phx", rom, pc)
def plx(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("plx", rom, pc)
def phy(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("phy", rom, pc)
def ply(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("ply", rom, pc)
# endregion

# region FLAGS
def clc(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("clc", rom, pc)
def sec(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("sec", rom, pc)
def cli(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("cli", rom, pc)
def sei(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("sei", rom, pc)
def clv(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("clv", rom, pc)
def cld(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("cld", rom, pc)
def sed(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("sed", rom, pc)
# endregion

# region JUMPS
def bpl(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_rel("bpl", rom, pc)
def bmi(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_rel("bmi", rom, pc)
def bvc(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_rel("bvc", rom, pc)
def bvs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_rel("bvs", rom, pc)
def bra(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_rel("bra", rom, pc)
def bcc(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_rel("bcc", rom, pc)
def bcs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_rel("bcs", rom, pc)
def bne(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_rel("bne", rom, pc)
def beq(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_rel("beq", rom, pc)


def jmp_abs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("jmp", rom, pc)
def jmp_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_ind("jmp", rom, pc)
def jmp_x_ind(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs_x_ind("jmp", rom, pc)


def jsr(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_abs("jsr", rom, pc)
def rti(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("rti", rom, pc)
def rts(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("rts", rom, pc)

def bbr0(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbr0", rom, pc)
def bbr1(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbr1", rom, pc)
def bbr2(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbr2", rom, pc)
def bbr3(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbr3", rom, pc)
def bbr4(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbr4", rom, pc)
def bbr5(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbr5", rom, pc)
def bbr6(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbr6", rom, pc)
def bbr7(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbr7", rom, pc)
def bbs0(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbs0", rom, pc)
def bbs1(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbs1", rom, pc)
def bbs2(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbs2", rom, pc)
def bbs3(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbs3", rom, pc)
def bbs4(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbs4", rom, pc)
def bbs5(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbs5", rom, pc)
def bbs6(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbs6", rom, pc)
def bbs7(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_zpg_rel("bbs7", rom, pc)
# endregion

# region TRANSFERS
def txs(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("txs", rom, pc)
def tsx(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("tsx", rom, pc)
def txa(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("txa", rom, pc)
def tax(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("tax", rom, pc)
def tya(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("tya", rom, pc)
def tay(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("tay", rom, pc)
# endregion

def wai(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("wai", rom, pc)
def stp(rom: bytes, pc: int) -> tuple[bytes, str]: return opc_imp("stp", rom, pc)

# endregion


opcode_table = [
#    -0         -1         -2         -3         -4         -5         -6         -7         -8         -9         -a         -b         -c         -d         -e         -f
    brk,       ora_x_ind, undefined, undefined, tsb_zpg,   ora_zpg,   asl_zpg,   rmb0,      php,       ora_imm,   asl_a,     undefined, tsb_abs,   ora_abs,   asl_abs,   bbr0, # 0-
    bpl,       ora_ind_y, ora_ind,   undefined, trb_zpg,   ora_zpg_x, asl_zpg_x, rmb1,      clc,       ora_abs_y, inc_a,     undefined, trb_abs,   ora_abs_x, asl_abs_x, bbr1, # 1-
    jsr,       and_x_ind, undefined, undefined, bit_zpg,   and_zpg,   rol_zpg,   rmb2,      plp,       and_imm,   rol_a,     undefined, bit_abs,   and_abs,   rol_abs,   bbr2, # 2-
    bmi,       and_ind_y, and_ind,   undefined, bit_zpg_x, and_zpg_x, rol_zpg_x, rmb3,      sec,       and_abs_y, dec_a,     undefined, bit_abs_x, and_abs_x, rol_abs_x, bbr3, # 3-
    rti,       eor_x_ind, undefined, undefined, undefined, eor_zpg,   lsr_zpg,   rmb4,      pha,       eor_imm,   lsr_a,     undefined, jmp_abs,   eor_abs,   lsr_abs,   bbr4, # 4-
    bvc,       eor_ind_y, eor_ind,   undefined, undefined, eor_zpg_x, lsr_zpg_x, rmb5,      cli,       eor_abs_y, phy,       undefined, undefined, eor_abs_x, lsr_abs_x, bbr5, # 5-
    rts,       adc_x_ind, undefined, undefined, stz_zpg,   adc_zpg,   ror_zpg,   rmb6,      pla,       adc_imm,   ror_a,     undefined, jmp_ind,   adc_abs,   ror_abs,   bbr6, # 6-
    bvs,       adc_ind_y, adc_ind,   undefined, stz_zpg_x, adc_zpg_x, ror_zpg_x, rmb7,      sei,       adc_abs_y, ply,       undefined, jmp_x_ind, adc_abs_x, ror_abs_x, bbr7, # 7-
    bra,       sta_x_ind, undefined, undefined, sty_zpg,   sta_zpg,   stx_zpg,   smb0,      dey,       bit_imm,   txa,       undefined, sty_abs,   sta_abs,   stx_abs,   bbs0, # 8-
    bcc,       sta_ind_y, sta_ind,   undefined, sty_zpg_x, sta_zpg_x, stx_zpg_y, smb1,      tya,       sta_abs_y, txs,       undefined, stz_abs,   sta_abs_x, stz_abs_x, bbs1, # 9-
    ldy_imm,   lda_x_ind, ldx_imm,   undefined, ldy_zpg,   lda_zpg,   ldx_zpg,   smb2,      tay,       lda_imm,   tax,       undefined, ldy_abs,   lda_abs,   ldx_abs,   bbs2, # a-
    bcs,       lda_ind_y, lda_ind,   undefined, ldy_zpg_x, lda_zpg_x, ldx_zpg_y, smb3,      clv,       lda_abs_y, tsx,       undefined, ldy_abs_x, lda_abs_x, ldx_abs_y, bbs3, # b-
    cpy_imm,   cmp_x_ind, undefined, undefined, cpy_zpg,   cmp_zpg,   dec_zpg,   smb4,      iny,       cmp_imm,   dex,       wai,       cpy_abs,   cmp_abs,   dec_abs,   bbs4, # c-
    bne,       cmp_ind_y, cmp_ind,   undefined, undefined, cmp_zpg_x, dec_zpg_x, smb5,      cld,       cmp_abs_y, phx,       stp,       undefined, cmp_abs_x, dec_abs_x, bbs5, # d-
    cpx_imm,   sbc_x_ind, undefined, undefined, cpx_zpg,   sbc_zpg,   inc_zpg,   smb6,      inx,       sbc_imm,   nop,       undefined, cpx_abs,   sbc_abs,   inc_abs,   bbs6, # e-
    beq,       sbc_ind_y, sbc_ind,   undefined, undefined, sbc_zpg_x, inc_zpg_x, smb7,      sed,       sbc_abs_y, ply,       undefined, undefined, sbc_abs_x, inc_abs_x, bbs7, # f-
]
