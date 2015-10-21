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

void key_down(uint8_t mod, uint8_t code) {
    uint8_t i;

    ep1_buf[0] = mod;  /* modifiers: R GUI, R ALT, R SHIFT, R CTRL,L GUI, L ALT, L SHIFT, L CTRL */

    for (i=2; i<8; i++) {
        if (!ep1_buf[i]) {
            ep1_buf[i] = code;
            ep1_cnt = i+1;
            return;
        }
    }
    /* If control flow reaches this point, there are more than 6 keys pressed. We don't have any provisions for that and
     * just ignore any additional keys pressed. */
}

void key_release(uint8_t mod, uint8_t code) {
    uint8_t i;

    ep1_buf[0] = mod; /* modifiers */

    /* find the entry in the current buffer, remove it and shift up any following entries. */
    for (i=2; i<8; i++)
        if (ep1_buf[i] == code)
            break;
    for (; i<8; i++) {
        uint8_t next = ep1_buf[i+1];
        ep1_buf[i] = next;
        if (!next) { /* will always happen at some iteration */
            ep1_cnt = i;
        }
    }
}

int main(void) {
    DDRD |= 0x30;

    usb_init_device();
    memset((void *)ep1_buf, 0, sizeof(ep1_buf));
    sei();
    PORTD |= 0x20;

    uint16_t i;
    while(23) {
        i++;
        if (!i)
            PORTD ^= 0x10;
    }
}
