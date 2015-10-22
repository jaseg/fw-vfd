/* Licence: GPLv3
 *
 * Copyright (c) 2013 Guy Weiler <weigu@weigu.lu>
 * Copyright (c) 2015 Sebastian Götte <jaseg@jaseg.net>
 *
 * This work modified from weigu www.weigu.lu
 * inspired by the lib from S. Salewski (http://http://www.ssalewski.de)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "usb_srs_hid.h"

#define NONE 0x00
#define LCTR 0xe0 /* left control */
#define LSHT 0xe1 /* left shift */
#define LALT 0xe2 /* left alt */
#define LGUI 0xe3 /* left gui */
#define RCTR 0xe4 /* right control */
#define RSHT 0xe5 /* right shift */
#define RALT 0xe6 /* right alt */
#define RGUI 0xe7 /* right gui */

struct kbd_hid_rep {
    uint8_t mod;
    uint8_t _pad;
    uint8_t keys[6];
};

volatile struct kbd_hid_rep *rep = (volatile struct kbd_hid_rep *)ep1_buf;

uint8_t modbyte(uint8_t code) {
    /* modifiers: R GUI, R ALT, R SHIFT, R CTRL,L GUI, L ALT, L SHIFT, L CTRL */
    switch (code) {
        case RGUI:
            return 0x80;
        case RALT:
            return 0x40;
        case RSHT:
            return 0x20;
        case RCTR:
            return 0x10;
        case LGUI:
            return 0x08;
        case LALT:
            return 0x04;
        case LSHT:
            return 0x02;
        case LCTR:
            return 0x01;
        default:
            return 0;
    }
}

void key_down(uint8_t code) {
    uint8_t i;

    if ((i = modbyte(code)))
        return rep->mod |= i, (void)0;

    for (i=0; i<sizeof(rep->keys); i++) {
        if (!rep->keys[i]) {
            rep->keys[i] = code;
            ep1_cnt = i+3;
            return;
        }
    }
    /* If control flow reaches this point, there are more than 6 keys pressed (or we were called with a NONE event). We
     * don't have any provisions for that and just ignore any additional keys pressed. The USB HID standard says we
     * should send an "rollover" code for all keys, but I don't like that. */
}

void key_up(uint8_t code) {
    uint8_t i;

    if ((i = modbyte(code)))
        return rep->mod &= ~i, (void)0;

    /* find the entry in the current buffer, remove it and shift up any following entries. */
    for (i=0; i<sizeof(rep->keys); i++)
        if (rep->keys[i] == code)
            break;

    for (; i<sizeof(rep->keys); i++) {
        uint8_t next = rep->keys[i+1];
        rep->keys[i] = next;
        if (!next) /* will always happen at some iteration */
            break;
    }
    ep1_cnt = i+2;
}

void virt_uart_tx(const char *s, uint8_t len) {
    memcpy((char *)ep3_buf, s, len);
    ep3_cnt = len;
    while (ep3_cnt);
}

char hexnibble(uint8_t f) {
    if (f<10)
        return f+'0';
    else
        return f-0xa+'a';
}

void type(char c) {
    uint8_t k = 0;
    if (c == '0')
        k = 0x27;
    else if ('1' <= c && c <= '9')
        k = 0x1e + c-'1';
    else if ('a' <= c && c <= 'f')
        k = 0x04 + c - 'a';
    else if (c == ' ')
        k = 0x2c;
    else if (c == ';')
        k = 0x33;
    else
        return;
    key_down(k);
    _delay_ms(20);
    key_up(k);
    _delay_ms(20);
}

void hid_dump(const uint8_t a, const uint8_t b, const uint8_t c, const uint8_t d) {
    type(hexnibble(a&0xf));
    type(hexnibble(a>>4));
    type(' ');
    type(hexnibble(b&0xf));
    type(hexnibble(b>>4));
    type(' ');
    type(hexnibble(c&0xf));
    type(hexnibble(c>>4));
    type(' ');
    type(hexnibble(d&0xf));
    type(hexnibble(d>>4));
    type(';');
}

int main(void) {
    DDRD  |= 0x30; /* Arduino TX/RX LEDs, active low */
    PORTD |= 0x20; /* TX LED */

    /* CAUTION! The keyboard slave interface uses 9N1 (!) framing. The 9th bit indicates key press (1) or release (0),
     * the remaining 8 bits the USB HID keycode. */
    DDRD  |= 0x08;
    UBRR1  = F_CPU/16/(BAUDRATE-1);
    UCSR1B = (1<<RXEN1);
    UCSR1C = (7<<UCSZ10) | (1<<USBS1);

    usb_init_device();
    memset((void *)ep1_buf, 0, sizeof(ep1_buf));
    sei();

    while(23) {
//        if (ep4_cnt) {
//        }
        uint8_t st = UCSR1A;
        if (!(st & (1<<RXC1)))
            continue;
        PORTD ^= 0x10; /* RX LED */
        uint8_t b = UCSR1B;
        uint8_t c = UCSR1C;
        hid_dump(st, b, c, UDR1);
/*        uint8_t _bar = UCSR1B;
        PORTD &= ~0x20;
        PORTD |= (_bar & 0x02) ? 0 : 0x20;
        uint8_t _foo = UDR1;
        key_down(_foo);
        _delay_ms(100);
        key_up(_foo);
        _delay_ms(1000);
        */
        continue;
//        if (st & ((1<<FE1) | (1<<DOR1) | (1<<UPE1))) /* some kind of error */
//            continue;
        if (UCSR1B & (1<<RXB81)) {
            uint8_t _foo = UDR1;
            key_down(0x04);
        } else {
            uint8_t _foo = UDR1;
            key_up(0x04);
        }
    }
}
