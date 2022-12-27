from typing import Tuple
from functools import cache

import gdb

from opcodes import opcode_table


def lookup_global_value(name: str, frame: gdb.Frame | None = None) -> gdb.Value:
    symbol = gdb.lookup_global_symbol(name)
    if symbol is None:
        raise ValueError
    if frame:
        return symbol.value(frame)
    return symbol.value()


@cache
def get_rom() -> list[int]:
    symbol = gdb.lookup_global_symbol("rom")
    if symbol is None:
        raise ValueError

    gdb_rom = symbol.value()
    gdb_rom_size = gdb_rom.type.sizeof

    rom = []
    for i in range(gdb_rom_size):
        rom.append(int(gdb_rom[i]))
    
    return rom


@cache
def disassemble_rom() -> list[Tuple[int, str, int]]:
    print("disassembling")
    rom = get_rom()
    rom_size = len(rom)

    disassembly = []
    i = 0

    while i < rom_size - 6: # skip interrupt vector
        (b, op) = opcode_table[rom[i]](rom, i)
        pc = i + 0x10000 - rom_size
        disassembly.append((pc, op, len(b)))
        i += len(b)

    return disassembly


reg_names = ("a", "x", "y", "p", "s", "pc")


class EmuCommand(gdb.Command):
    def __init__(self) -> None:
        super().__init__("emu", gdb.COMMAND_USER)
        self.commands = {
            "registers",
            "list"
        }

    def invoke(self, argument: str, from_tty: bool) -> None:
        match gdb.string_to_argv(argument):
            case ["registers", *args]:
                self._emu_registers(*args)

            case ["list", *n]:
                self._emu_list(*n)

            case _:
                print(f"Unknown command {argument}")

    def complete(self, text: str, word: str) -> object:
        completions = []
        for command in self.commands:
            if command.startswith(word):
                completions.append(command)
        return completions

    def _emu_registers(self, *args):
        frame = gdb.selected_frame()
        def print_reg(name: str):
            reg = lookup_global_value(name, frame)
            print(f"{name}\t{hex(reg)}\t{reg}")

        if args == []:
            args = reg_names

        for reg in args:
            if reg not in reg_names:
                print(f"Invalid register '{reg}'")
                return
            print_reg(reg)

    def _emu_list(self, *n):
        if len(n) > 1:
            print(f"Invalid arguments {' '.join(n)}")
            return

        if len(n) == 1:
            n = int(n[0])
        else:
            n = 4

        pc = lookup_global_value("pc", gdb.selected_frame()) - 1

        rom = disassemble_rom()

        i = 0
        while rom[i][0] != pc:
            i += 1

        b = i if i < n else n - 1

        for (a, op, _) in rom[i-b:i+n]:
            print(f"{'>' if a == pc else ' '} {a:04x}  {op}")
        

EmuCommand()
