import traceback
from functools import cache
from dataclasses import dataclass
from collections import namedtuple

import gdb

from opcodes import opcode_table

@cache
def get_rom() -> bytes:
    symbol = gdb.lookup_global_symbol("rom")
    if symbol is None:
        raise ValueError

    gdb_rom = symbol.value()
    gdb_rom_size = gdb_rom.type.sizeof

    rom = bytearray()
    for i in range(gdb_rom_size):
        rom.append(int(gdb_rom[i]))

    return bytes(rom)


# @dataclass(slots=True)
# class Instruction:
#     address: int
#     opcode: bytes
#     mnemonic: str

#     def __iter__(self):
#         return iter((self.address, self.opcode, self.mnemonic))


Instruction = namedtuple('Instruction', ['address', 'opcode', 'mnemonic'])


class Disassembler:
    def __init__(self) -> None:
        self._disassemble()

    def _disassemble(self):
        rom = get_rom()
        rom_size = len(rom)

        i = 0
        address = 0x10000 - rom_size
        disassembly: list[Instruction] = []
        while i < rom_size - 6: # skip interrupt vector
            (opcode, mnemonic) = opcode_table[rom[i]](rom, i)
            i += len(opcode)
            disassembly.append(Instruction(address, opcode, mnemonic))
            address += len(opcode)

        self._dis_list = disassembly
        self._dis_dict = {instr.address: (i, instr) for (i, instr) in enumerate(disassembly)}

    def disassemble(self) -> list[Instruction]:
        return self._dis_list

    def at(self, index: int) -> Instruction:
        return self._dis_list[index]

    def index(self, address) -> int:
        return self._dis_dict[address][0]

    def __getitem__(self, address: int) -> Instruction:
        return self._dis_dict[address][1]


reg_names = ("a", "x", "y", "p", "s", "pc")


class EmuCommand(gdb.Command):
    def __init__(self) -> None:
        super().__init__("emu", gdb.COMMAND_USER, gdb.COMPLETE_NONE, True)


class EmuRegistersCommand(gdb.Command):
    def __init__(self):
        super().__init__("emu registers", gdb.COMMAND_USER, gdb.COMPLETE_NONE)

    def invoke(self, argument: str, from_tty: bool) -> None:
        argv = gdb.string_to_argv(argument)

        def print_reg(name: str):
            reg = gdb.parse_and_eval(name)
            print(f"{name}\t{hex(reg)}\t{reg}")

        if argv == []:
            argv = reg_names

        for reg in argv:
            if reg not in reg_names:
                print(f"Invalid register '{reg}'")
                return
            print_reg(reg)


breakpoint_spec = "main.c:15"
disassembler = Disassembler()


class EmuListCommand(gdb.Command):
    def __init__(self):
        super().__init__("emu list", gdb.COMMAND_USER, gdb.COMPLETE_NONE)

    def invoke(self, argument: str, from_tty: bool) -> None:
        pc = gdb.parse_and_eval("pc") - 1
        match gdb.string_to_argv(argument):
            case []:
                address = int(pc)
                n = 5
            case [i] if i.isdigit():
                address = int(pc)
                n = int(i)
            case [address, *i] if address.startswith('*$') and len(i) <= 1:
                address = int(address.removeprefix('*$'), 16)
                n = int(*i) if i else 5
            case _:
                print(("Invalid arguments", argument))
                return

        if (address < 0):
            i = 0
        else:
            i = disassembler.index(address)

        rom = disassembler.disassemble()
        for (address, bytes, mnemonic) in rom[i:i+n]:
            print(f"{'>' if address == pc else ' '}  ${address:04x}\t {' '.join(map(lambda b: f'{b:02x}', bytes)):8}\t {mnemonic}")


class EmuBreakpointCommand(gdb.Command):
    def __init__(self):
        super().__init__("emu breakpoint", gdb.COMMAND_USER, gdb.COMPLETE_NONE)

    def invoke(self, argument: str, from_tty: bool) -> None:
        self.dont_repeat()
        match gdb.string_to_argv(argument):
            case [addr]:
                addr = int(addr, 16) + 1
                breakpoint = gdb.Breakpoint(breakpoint_spec, internal=False)
                breakpoint.condition = f"$r11 == 0x{addr >> 8:02x} && $r10 == 0x{addr & 0xff:02x}"
                breakpoint.commands = '\n'.join(["emu list 1", "print p"])
            case _:
                print(("Invalid argument", argument))
                return


class EmuNextCommand(gdb.Command):
    def __init__(self):
        super().__init__("emu next", gdb.COMMAND_USER, gdb.COMPLETE_NONE)

    def invoke(self, argument: str, from_tty: bool) -> None:
        if argument != "":
            print(("Invalid argument", argument))
            return

        pc = int(gdb.parse_and_eval("pc")) - 1

        breakpoint = gdb.Breakpoint(breakpoint_spec, temporary=True, internal=True)
        breakpoint.commands = '\n'.join(["emu list 1", "print p"])

        instr = disassembler[pc]

        if instr.opcode[0] == 0x20: # jsr
            addr = disassembler[pc + len(instr.opcode)].address + 1
            breakpoint.condition = f"$r11 == 0x{addr >> 8:02x} && $r10 == 0x{addr & 0xff:02x}"

        gdb.execute("continue")


class EmuStepCommand(gdb.Command):
    def __init__(self):
        super().__init__("emu step", gdb.COMMAND_USER, gdb.COMPLETE_NONE)

    def invoke(self, argument: str, from_tty: bool) -> None:
        if argument != "":
            print(("Invalid argument", argument))
            return

        breakpoint = gdb.Breakpoint(breakpoint_spec, temporary=True, internal=True)
        breakpoint.commands = '\n'.join(["emu list 1", "print p"])
        gdb.execute("continue")


EmuCommand()
EmuRegistersCommand()
EmuListCommand()
EmuBreakpointCommand()
EmuNextCommand()
EmuStepCommand()
