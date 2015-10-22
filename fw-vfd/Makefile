
all: main.elf

font.c: fontconv.py
	./fontconv.py > font.c

main.elf: main.c font.c escape_handling.c
	avr-gcc -O2 -mmcu=atmega16 -DF_CPU=8000000 -DBAUDRATE=250000 -std=gnu11 -o $@ $^
	avr-size main.elf

clean:
	rm -f main.elf

.PHONY: clean

