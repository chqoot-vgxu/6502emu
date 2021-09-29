
release:
	@avr-gcc -Os -Wall -Wextra -mmcu=atmega328p main.c -o main.elf

upload: release
	@avrdude -c arduino -p atmega328p -P /dev/ttyACM0 -b 115200 -D -U flash:w:main.elf -v