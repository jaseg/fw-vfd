
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

int8_t keystate[90];

#define DEBOUNCE_TIME 20 /* roughly milliseconds */


#define NONE 0x00
#define LCTR 0xe0 /* left control */
#define LSHT 0xe1 /* left shift */
#define LALT 0xe2 /* left alt */
#define LGUI 0xe3 /* left gui */
#define RCTR 0xe4 /* right control */
#define RSHT 0xe5 /* right shift */
#define RALT 0xe6 /* right alt */
#define RGUI 0xe7 /* right gui */


const char keymap[90] = {
/*          0     1     2     3      4     5     6     7                8     9     a     b      c     d     e     f */
/* 0x00 */ 0x0f, 0x5a, 0x5b, 0x9a,  0x44, 0x18, 0x2e, 0x17, /* 0x08 */ 0x42, 0x1b, 0x12, 0x5f,  0x5e, 0x40, 0x3d, 0x11,
/* 0x10 */ 0x34, 0x22, 0x2a, 0x20,  0x25, 0x14, NONE, NONE, /* 0x18 */ NONE, 0x0a, 0x27, 0x08,  RCTR, 0x4e, 0x0d, LSHT,
/* 0x20 */ 0x1e, NONE, NONE, 0x23,  0x37, 0x07, 0x89, 0x04, /* 0x28 */ 0x10, NONE, NONE, NONE,  NONE, 0x05, 0x13, 0x06,
/* 0x30 */ 0x30, 0x1f, 0x0c, 0x43,  0x5d, NONE, NONE, 0x1c, /* 0x38 */ 0x33, 0x21, 0x28, LCTR,  0x26, 0x62, 0x60, NONE,
/* 0x40 */ 0x3b, 0x2c, 0x2d, 0x15,  RALT, 0x1d, 0x0e, 0x59, /* 0x48 */ 0xb0, 0x3e, 0x3a, 0x24,  NONE, 0x09, 0x35, 0x1a,
/* 0x50 */ 0x36, 0x5c, 0x61, 0x3f,  0x3c, 0x0b, 0x2f, 0x19, /* 0x58 */ LGUI, 0x16

/* * The caret ^ key has been assigned to the backtick/tilde key not present on this keyboard, and thus moved from left
 *   to right.
 * * The :* key has been assigned to keycode 0x89, "International 3" that is normally used for the Yen | _ key on
 *   japanese keyboards since that key has the same physical position.
 * * The vertical tab key has been mapped to 0x4e, "Page Down"
 * * The horizontal tab keys and the modifier [1] key have been assigned the left control, right control and right alt
 *   roles, respectively.
 * * The funky arrow key ⇻ on the right has been assigned the LGUI/Meta/Mod3/Windows key modifer role.
 * * The keypad P key has been assigned 0x9a "Keyboard SysReq/Attention". It has an additional spring and is harder to
 *   actuate than all other keys on the keyboard.
 * * The remaining red keypad keys have been assigned "Keyboard F" key mappings according to the following table
 *     1 F1  0x3a
 *     2 F2  0x3b
 *     3 F3  0x3c
 *     3 F4  0x3d
 *     4 F5  0x3e
 *     5 F6  0x3f
 *     6 F7  0x40
 *     7 F8  0x41
 *     c F9  0x42
 *     r F10 0x43
 *     e F11 0x44
 * * The keyboard does not distinguish between left and right shift keys (caused by its wiring which I did not want to
 *   modify). For both keys, the sent modifer keycode is 0xe1 "Left Shift"
 */
};


/* CAUTION! The UART uses 9N1 frames !! */
void report_down (uint8_t c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UCSR0B |= (1<<TXB80);
    UDR0 = c;
}

void report_up (uint8_t c) {
    while (!(UCSR0A & (1<<UDRE0)));
    UCSR0B &= ~(1<<TXB80);
    UDR0 = c;
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
      *
      * The device outputs 9N1 (!) data on its uart. The 9th bit indicates key press (1) or release (0), the remaining 8
      * bits the USB HID keycode.
      */
    DDRB   = 0x0f; /* row outputs, shift */

    /* uart setup */
    DDRD  |= 0x02;
    UBRR0  = F_CPU/16/(BAUDRATE-1);
    UCSR0B = (1<<TXEN0);
    UCSR0C = (7<<UCSZ00) | (1<<USBS0);

    int ridx = 0;
    reset_row();
    while (23) {
        uint16_t cols = read_cols();

        uint8_t cidx = ridx;
        if (ridx == 30)
            cols |= (PINB&1)<<1; /* map hard shift to keycode 0x1f (31) */
        while (cidx < ridx+10) {
            if (keystate[cidx] > 1) {
                keystate[cidx]--;
            } else if (keystate[cidx] < 0) {
                keystate[cidx]++;
            } else {
                uint8_t c = keymap[cidx];
                if (cols & 1) {
                    if (!keystate[cidx]) {
                        keystate[cidx] = DEBOUNCE_TIME;
                        report_down(c);
                    }
                } else {
                    if (keystate[cidx]) {
                        keystate[cidx] = -DEBOUNCE_TIME;
                        report_up(c);
                    }
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

