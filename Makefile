
debug:
	@avr-gcc -O0 -g -Wall -Wextra -mmcu=atmega328p main.c -o maind.elf

simavr: debug
	@simavr -f 16000000 -m atmega328p -g -v maind.elf

release:
	@avr-gcc -Os -Wall -Wextra -mmcu=atmega328p main.c -o main.elf

upload: release
	@avrdude -c arduino -p atmega328p -P /dev/ttyACM0 -b 115200 -D -U flash:w:main.elf -v

clean:
	rm main.elf maind.elf
