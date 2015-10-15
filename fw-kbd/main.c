
#include <stdint.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>


uint16_t read_cols () {
    /* returns the 10-bit value read from column inputs.
     * Mapping of 10-bit output to original KR3600 column inputs:
     *
     * MSB                                               LSB
     *  |        [%B0]           |          [%A0]         |
     * [0  0  0  0   0  0  Y8 Y9   Y7 Y6 Y4 Y3  Y2 Y1 Y0 Y5]
     */
    return ((PINB&0x30) << 4) | ((PIND&0x0c) << 4) | (PINC&0x3f);
}

void reset_row (void) {
    PORTB &= ~0x0e;
    PORTB |= 0x08; /* X9/!OE */
    PORTB |= 0x02; /* DQ */
    PORTB |= 0x04; /* shift/strobe */
    _delay_us(10);
    PORTB &= ~0x04; /* shift/strobe */
    PORTB &= ~0x02; /* DQ */
}

void next_row(void) {
    PORTB &= ~0x08; /* X9/!OE */
    PORTB |= 0x04; /* shift/strobe */
    _delay_us(10);
    PORTB &= ~0x04;
}

char to_hex(uint8_t nibble) {
    if (nibble < 0xa)
        return '0'+nibble;
    else
        return 'a'+(nibble-0xa);
}

void uart_putc(char c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UDR0 = c;
}

uint8_t keystate[90];

#define DEBOUNCE_TIME 20 /* roughly milliseconds */

const char keymap[90] = {
/*          0     1     2     3      4     5     6     7                8     9     a     b      c     d     e     f */
/* 0x00 */ 'l',  0x93, 0x94, 0x99,  0x98, 'u',  0x81, 't',  /* 0x08 */ 0x8a, 'x',  'o',  0x8c,  0x91, 0x9d, 0x9a, 'n',
/* 0x10 */ 0x85, '5',  '\b', '3',   '8',  'q',  '\e', '\e', /* 0x18 */ '\e', 'g',  '0',  'e',   0x88, 0x82, 'j',  0x9a,
/* 0x20 */ '1',  '\e', '\e', '6',   '.',  'd',  '*',  'a',  /* 0x28 */ 'm',  '\e', '\e', '\e',  '\e', 'b',  'p',  'c',
/* 0x30 */ ';',  '2',  'i',  0x8b,  0x90, '\e', '\e', 'z',  /* 0x38 */ 0x84, '4',  '\n', 0x87,  '9',  0x95, 0x8d, '\e',
/* 0x40 */ 0x98, ' ',  '-',  'r',   0x89, 'y',  'k',  0x92, /* 0x48 */ 0x96, 0x9b, 0x97, '7',   '\e', 'f',  '^',  'w',
/* 0x50 */ ',',  0x8f, 0x8e, 0x9c,  0x99, 'h',  0x83, 'v',  /* 0x58 */ 0x86, 's'

/* 0x81 §   [paragraph]
 * 0x82 ⇟   [page down]
 * 0x83 ü   [u w/ diaresis]
 * 0x84 ö   [o w/ diaresis]
 * 0x85 ä   [a w/ diaresis]
 * 0x86 ⇻   [right]
 * 0x87 ⇤   [tab left]
 * 0x88 ⇤   [tab right]
 * 0x89 [1] [mod 1 key]
 *
 * 0x8a NC
 * 0x8b NR
 * 0x8c N7
 * 0x8d N8
 * 0x8e N9
 * 0x8f N4
 * 0x90 N5
 * 0x91 N6
 * 0x92 N1
 * 0x93 N2
 * 0x94 N3
 * 0x95 N0
 * 0x96 N00
 * 0x97 NR1
 * 0x98 NR2
 * 0x99 NR3
 * 0x9a NR4
 * 0x9b NR5
 * 0x9c NR6
 * 0x9d NR7
 * 0x98 NE
 * 0x99 NP
 * 0x9a Shift
 */
};

void report (uint8_t idx) {
    char c = keymap[idx];
    uart_putc(c);
}


int main(void) {
     /* pin map:
      *
      * PB0 hardware shift (r/l, TODO: split r/l)
      * PB1 row data out (to 74HC595)
      * PB2 row shift (same)
      * PB3 row X8 direct output
      * PB4 col in  Y9
      * PB5 col in  Y8
      * PB6 XTAL
      * PB7 XTAL
      *
      * PC0 col in  Y5
      * PC1 col in  Y0
      * PC2 col in  Y1
      * PC3 col in  Y2
      * PC4 col in  Y3
      * PC5 col in  Y4
      * PC6 !RESET
      *
      * PD0 RXD
      * PD1 TXD
      * PD2 col in  Y6
      * PD3 col in  Y7
      * PD4 NC
      * PD5 NC
      * PD6 NC
      * PD7 NC
      *
      * We're not un-scrambling the rows here, we are just using them directly to index the lookup table.
      */
    DDRB   = 0x0f; /* row outputs, shift */

    /* uart setup */
    DDRD  |= 0x02;
    UBRR0  = F_CPU/16/(BAUDRATE-1);
    UCSR0B = (1<<TXEN0);
    UCSR0C = (3<<UCSZ00);

    int ridx = 0;
    reset_row();
    while (23) {
        uint16_t cols = read_cols();

        uint8_t cidx = ridx;
        if (ridx == 30)
            cols |= (PINB&1)<<1; /* map hard shift to keycode 0x1f (31) */
        while (cidx < ridx+10) {
            if (keystate[cidx]&0x80) {
                keystate[cidx]--;
            } else {
                if (cols & 1) {
                    if (!keystate[cidx]) {
                        keystate[cidx] = 0x80 | DEBOUNCE_TIME;
                        report(cidx);
                    }
                } else {
                    keystate[cidx] = 0;
                }
            }
            cidx ++;
            cols >>= 1;
        }

        if (ridx == 80) {
            ridx = 0;
            reset_row();
        } else {
            ridx += 10;
            next_row();
        }
        _delay_us(100);
    }
}

