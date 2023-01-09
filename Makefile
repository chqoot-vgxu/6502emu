# TODO: learn to write makefiles

CC := avr-gcc
CFLAGS := -mmcu=atmega328p -mrelax -fdata-sections -ffunction-sections -Wl,--gc-sections \
          -Wall -Wextra -Wno-array-bounds -Wno-unused-parameter \
		  -Ibuild/generated

SIMAVR_FLAGS = -f 16000000 -m atmega328p -v
ifeq ($(MODE),debug)
	CFLAGS += -Og -g
	SIMAVR_FLAGS += -g
else
	MODE := release
	CFLAGS += -Os -g
endif

SOURCE_DIR := .
BUILD_DIR  := build
GEN_DIR    := $(BUILD_DIR)/generated

ROM_FILE := $(BUILD_DIR)/$(notdir $(ASM:.asm=.bin))
HEADERS  := $(GEN_DIR)/rom.h $(wildcard *.h)
SOURCES  := $(GEN_DIR)/rom.c $(wildcard *.c)
OBJECTS  := $(addprefix $(BUILD_DIR)/$(MODE)/, $(notdir $(SOURCES:.c=.o)))

6502emu: $(BUILD_DIR)/$(MODE)/6502emu | $(BUILD_DIR)

$(BUILD_DIR): | $(GEN_DIR) $(BUILD_DIR)/$(MODE)

$(GEN_DIR):
	mkdir -p $(GEN_DIR)

$(BUILD_DIR)/$(MODE):
	mkdir -p $(BUILD_DIR)/$(MODE)

$(BUILD_DIR)/$(MODE)/6502emu: $(OBJECTS) | $(BUILD_DIR)
	$(CC) $(CFLAGS) $^ -o $@

$(ROM_FILE): $(ASM) | $(BUILD_DIR)
ifeq ($(ASM),)
	$(error Assembly file not defined)
endif
	@- rm $(BUILD_DIR)/*.bin
	vasm6502_oldstyle -w -Fbin -dotdir -esc -wdc02 $(ASM) -o $@

$(BUILD_DIR)/$(MODE)/rom.o: $(ROM_FILE) | $(BUILD_DIR)
	./utils/bin2header.py $^
	$(CC) -c $(CFLAGS) -o $@ $(GEN_DIR)/rom.c

$(BUILD_DIR)/$(MODE)/main.o: main.c opcodes.h | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(MODE)/opcodes.o: opcodes.c opcodes.h $(GEN_DIR)/rom.c $(GEN_DIR)/rom.h | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) $< -o $@

upload: $(BUILD_DIR)/$(MODE)/6502emu
	avrdude -c arduino -p atmega328p -P /dev/ttyACM0 -b 115200 -D -U flash:w:$(BUILD_DIR)/$(MODE)/6502emu -v

run: 6502emu
	simavr $(SIMAVR_FLAGS) $(BUILD_DIR)/$(MODE)/6502emu

clean:
	- rm -r $(BUILD_DIR)

.PHONY: 6502emu upload run clean
