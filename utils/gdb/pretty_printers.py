import gdb

from opcodes import opcode_table


class OpcodePrinter:
    def __init__(self, opc) -> None:
        self.opc = int(opc)

    def to_string(self) -> str:
        return opcode_table[self.opc].__name__

    def display_hint(self):
        return 'string'


class FlagsPrinter:
    def __init__(self, p) -> None:
        self.p = int(p)

    def to_string(self) -> str:
        flags = ['C', 'Z', 'I', 'D', 'B', '-', 'V', 'N']
        for i in range(8):
            if not self.p & (1 << i):
                flags[i] = '-'
        return ''.join(reversed(flags))

    def display_hint(self):
        return 'string'


def build_pretty_printer():
    pp = gdb.printing.RegexpCollectionPrettyPrinter("6502emu")
    pp.add_printer('opcode', 'opcode_t', OpcodePrinter)
    pp.add_printer('flags',  'flags_t',  FlagsPrinter)
    return pp


gdb.printing.register_pretty_printer(
    gdb.current_objfile(),
    build_pretty_printer(),
    replace=True)
