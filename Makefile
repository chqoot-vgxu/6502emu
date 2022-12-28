# TODO: learn to write makefiles

CC := avr-gcc
CFLAGS := -Os -Wall -Wextra -Wno-array-bounds -Wno-unused-parameter -mmcu=atmega328p -mrelax -fdata-sections -ffunction-sections -Wl,--gc-sections

ifeq ($(MODE),debug)
	CFLAGS += -Og -g
	BUILD_DIR := build/debug
else
	MODE := release
	CFLAGS += -Os
	BUILD_DIR := build/release
endif

ifeq ($(ASM),)
$(error Assembly file not defined)
endif

SOURCE_DIR := .
BUILD_DIR  := build

ROM_FILE := $(BUILD_DIR)/$(notdir $(ASM:.asm=.bin))
HEADERS  := $(wildcard *.h)
SOURCES  := $(wildcard *.c)
OBJECTS  := $(addprefix $(BUILD_DIR)/$(MODE)/, $(notdir $(SOURCES:.c=.o)))

6502emu: $(BUILD_DIR)/$(MODE)/rom.o $(OBJECTS)
	@ mkdir -p build
	$(CC) $(CFLAGS) $^ -o $@.elf

$(ROM_FILE):
	@ mkdir -p $(BUILD_DIR)/$(MODE)
	vasm6502_oldstyle -w -Fbin -dotdir -esc -wdc02 $(ASM) -o $@

rom.c: $(ROM_FILE)
	@ mkdir -p $(BUILD_DIR)/$(MODE)
	./utils/bin2header.py $^

$(BUILD_DIR)/$(MODE)/rom.o: rom.c
	@ mkdir -p $(BUILD_DIR)/$(MODE)
	$(CC) -c $(CFLAGS) -o $@ $^

$(BUILD_DIR)/$(MODE)/%.o: $(SOURCE_DIR)/%.c
	@ mkdir -p $(BUILD_DIR)/$(MODE)
	$(CC) -c $(CFLAGS) -o $@ $<

upload: 6502emu
	avrdude -c arduino -p atmega328p -P /dev/ttyACM0 -b 115200 -D -U flash:w:6502emu.elf -v

clean:
	-rm -r $(BUILD_DIR) *.elf rom.*

.PHONY: 6502emu upload clean
