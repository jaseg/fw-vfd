/* Licence: GPL-3
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
#include "usb_srs_hid.h"

void sendkey(uint8_t mod, uint8_t code) {
    Ep1_cnt = 3; // 3 bytes
    cli();
    // 1 Byte = key modifier Byte (7-0):
    // R GUI, R ALT, R SHIFT, R CTRL,L GUI, L ALT, L SHIFT, L CTRL
    Ep1_buf[0] = mod;  // modifier byte
    Ep1_buf[1] = 0x00; // reserved byte = 0
    Ep1_buf[2] = code; // scan code (up to 6 keys)
    sei();
    Ep1_flag = 1;  // set EP1 flag
    _delay_ms(50); // higher than bInterval in Endpoint Descriptor!
    Ep1_cnt = 3;   // 3 bytes
    cli();
    Ep1_buf[0] = 0x00; // modifier byte
    Ep1_buf[1] = 0x00; // reserved byte = 0
    Ep1_buf[2] = 0x00; ////stop pressing key
    sei();
    Ep1_flag = 1; // set EP1 flag
    _delay_ms(3); // higher than bInterval in Endpoint Descriptor!
}

int main(void) {
    usb_init_device();
    while(23);
}
