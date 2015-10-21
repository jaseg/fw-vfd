
MCU 	 = atmega8u2
F_CPU  	 = 16000000
BAUDRATE = 9600

CFLAGS := -mmcu=$(MCU) -DF_CPU=$(F_CPU)UL -DBAUDRATE=$(BAUDRATE) -O2 -std=gnu99 -Wall -funsigned-char -funsigned-bitfields -fpack-struct -fshort-enums

all: main.elf

program: main.elf
	dfu-programmer $(MCU) flash $(TARGET).hex
	dfu-programmer $(MCU) start

main.elf: main.c usb_srs_hid.c
	avr-gcc $(CFLAGS) -o $@ $^
	avr-size $@

lst: main.lst usb_srs_hid.lst

%.lst: %.c
	avr-gcc -c -g -Wa,-a,-ad $(CFLAGS) $^ > $@

clean:
	rm -f main.elf 

.PHONY:	program clean lst
