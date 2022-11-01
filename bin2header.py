#!/usr/bin/python

import os, sys


def main(bin_name: str, header_name: str | None):
    if not os.path.isfile(bin_name):
        print(f"The file {bin_name} doesn't exists", file=sys.stderr)
        return 1

    if header_name is None:
        header_name = os.path.splitext(bin_name)[0] + '.h'


    with open(bin_name, "rb") as bin, open(header_name, "w") as header:
        header.write(f"""\
/* GENERATED FROM '{bin_name}' */
#pragma once

#include <stdint.h>
#include <avr/pgmspace.h>

const uint8_t PROGMEM rom[] = {{
""")

        chars = tuple(ord(c) for c in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890'!\"£$%&/()=?^,.-+;:_*#[]\\|/@` ")
        def char(c: int) -> str:
            if c in chars:
                return chr(c)
            return '.'

        address = 0
        while (chunk := bin.read(16)):
            header.write(f"/*{address:04X}*/  {', '.join(f'0x{b:02X}' for b in chunk)},  /* {''.join(char(b) for b in chunk)} */\n")
            address += 16

        header.write('};\n')


if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='input file name', type=str)
    parser.add_argument('-o', '--output', help='output file name', type=str, default=None)
    args = parser.parse_args()

    exit(main(args.input, args.output))
