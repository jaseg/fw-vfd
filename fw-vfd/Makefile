
all: main.elf


main.elf: main.c
	avr-gcc -O2 -mmcu=atmega16 -DF_CPU=8000000 -DBAUDRATE=9600 -std=gnu11 -o $@ $^

clean:
	rm -f main.elf

.PHONY: clean

