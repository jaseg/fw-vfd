
all: main.elf


main.elf: main.c
	avr-gcc -O2 -mmcu=atmega328p -DF_CPU=16000000 -DBAUDRATE=9600 -o $@ $^

