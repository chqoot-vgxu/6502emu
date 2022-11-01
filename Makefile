# TODO: learn to write makefiles

debug:
	@avr-gcc -O0 -g -Wall -Wextra -Wconversion -Wno-unused-parameter -mmcu=atmega328p main.cpp -o maind.elf

simavr: debug
	@simavr -f 16000000 -m atmega328p -g -v maind.elf

release:
	@avr-gcc -Os -Wall -Wextra -Wconversion -Wno-unused-parameter -mmcu=atmega328p main.cpp -o main.elf

upload: release
	@avrdude -c arduino -p atmega328p -P /dev/ttyACM0 -b 115200 -D -U flash:w:main.elf -v

clean:
	rm main.elf maind.elf
