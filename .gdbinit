source utils/gdb/commands.py
source utils/gdb/pretty_printers.py

break main

break main.c:15
commands
    emu list 1
    emu registers p
end

#target remote :1234
