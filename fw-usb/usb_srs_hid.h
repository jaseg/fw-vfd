/* Licence: GPLh3
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

#ifndef _USB_SRS_HID_H_
#define _USB_SRS_HID_H_

#include <avr/io.h>
#include <stdint.h>

extern volatile uint8_t ep1_buf[16]; /* TODO about 8 bytes should be sufficient here. Check USB FIFO handling and call sites. */
extern volatile uint8_t ep1_cnt;

extern volatile uint8_t ep2_buf[16];
extern volatile uint8_t ep2_cnt;

extern volatile uint8_t ep3_buf[64];
extern volatile uint8_t ep3_cnt;

extern volatile uint8_t ep4_buf[64];
extern volatile uint8_t ep4_cnt;

void usb_init_device(void);
void usb_init_endpoint(uint8_t ep, uint8_t type, uint8_t direction, uint8_t size, uint8_t banks);
void usb_send_descriptor(const uint8_t *d, uint8_t len);

void usb_ep0_setup(void);
void usb_ep_in(volatile uint8_t *buf, volatile uint8_t *count);
void usb_ep_out(volatile uint8_t *buf, volatile uint8_t *count);

#endif
