#!/usr/bin/python

import os, sys


def main(bin_name: str):
    if not os.path.isfile(bin_name):
        print(f"The file {bin_name} doesn't exists", file=sys.stderr)
        return 1

    gen_dir_name = os.path.join('build', 'generated') 
    header_name = os.path.join(gen_dir_name, 'rom.h')
    source_name = os.path.join(gen_dir_name, 'rom.c')

    bin_stat = os.stat(bin_name)


    with open(bin_name, "rb") as bin:
        size = 0
        address = 0xffff - bin_stat.st_size + 1

        with open(source_name, "w") as source:

            chars = tuple(ord(c) for c in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ1234567890'!\"£$%&/()=?^,.-+;:_*#[]\\|/@` ")
            def char(c: int) -> str:
                if c in chars:
                    return chr(c)
                return '.'

            source.write(f"""\
/* GENERATED FROM '{bin_name}' */
#include "rom.h"
#include <avr/pgmspace.h>

const uint8_t PROGMEM rom[ROM_SIZE] = {{
/*          0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F                        */
""")

            while (chunk := bin.read(16)):
                source.write(f"/*{address:04X}*/  {', '.join(f'0x{b:02X}' for b in chunk)},  /* {''.join(char(b) for b in chunk)} */\n")
                size += 16
                address += 16

            source.write('};\n')

        with open(header_name, "w") as header:
            header.write(f"""\
/* GENERATED FROM '{bin_name}' */
#pragma once

#include <stdint.h>

#define ROM_SIZE {size}
extern const uint8_t rom[ROM_SIZE];
""")



if __name__ == '__main__':
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument('input', help='input file name', type=str)
    args = parser.parse_args()

    exit(main(args.input))
