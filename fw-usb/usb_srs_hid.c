/* Licence: GPLv3
 *
 * Copyright (c) 2009 Guy Weiler <weigu@weigu.lu>
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

#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include "usb_srs_hid.h"

#define UNUSED(x) (void)(x)


volatile uint8_t ep1_buf[64]; /* TODO about 8 bytes should be sufficient here. Check USB FIFO handling and call sites. */
volatile uint8_t ep1_cnt;

volatile uint8_t ep2_buf[64];
volatile uint8_t ep2_cnt;

volatile uint8_t ep3_buf[64];
volatile uint8_t ep3_cnt;

volatile uint8_t ep4_buf[64];
volatile uint8_t ep4_cnt;


ISR(USB_GEN_vect) {
    if (UDINT & (1 << EORSTI)) { // End Of ReSeT?
        UDINT &= ~(1<<EORSTI);      // sperre EORSTI
        usb_init_endpoint(0, 0, 0, 0, 0); /* control out, 8 bytes, 1 bank */
        UEIENX |= (1<<RXSTPE);
    }
}

ISR(USB_COM_vect) {
    if (UEINT&1) { /* ep 0 */
        UENUM = 0;
        if (UEINTX & (1<<RXSTPI))
            usb_ep0_setup();
    }

    if (UEINT&2) { /* ep 1 */
        UENUM = 1;
        usb_ep_in(ep1_buf, &ep1_cnt);
    }

    if (UEINT&4) { /* ep 2 */
        UENUM = 2;
        usb_ep_out(ep2_buf, &ep2_cnt, sizeof(ep2_buf));
    }

    if (UEINT&8) { /* ep 3 */
        UENUM = 3;
        usb_ep_in(ep3_buf, &ep3_cnt);
    }

    if (UEINT&16) { /* ep 4 */
        UENUM = 4;
        usb_ep_out(ep4_buf, &ep4_cnt, sizeof(ep4_buf));
    }
}


void usb_init_device(void) {
    USBCON = (1<<USBE) | (1<<FRZCLK);

    /* toggle FRZCLK to wake up module */
    USBCON &= ~(1<<FRZCLK);
    USBCON |= (1<<FRZCLK);
    /* start USB PLL */
    PLLCSR = 0x04;
    PLLCSR |= (1<<PLLE);

    while (!(PLLCSR & (1<<PLOCK)))
        ; /* wait for PLL to lock */

    USBCON &= ~(1<<FRZCLK); /* enable clock */
    UDCON   = 0;            /* attach device */
    UDIEN   = (1<<EORSTE);  /* EndOfReSeT interrupt */

    ep1_cnt = 0;
    ep2_cnt = 0;
}

void usb_init_endpoint(uint8_t ep, uint8_t type, uint8_t direction, uint8_t size, uint8_t banks) {
    UENUM = ep;
    UECONX |= (1<<EPEN);
    UECFG0X = ((type<<6) | direction);
    UECFG1X = ((size<<4) | (banks<<2));
    UECFG1X |= (1<<ALLOC);
}


static const uint8_t PROGMEM dev_des[] = {
    18,         // bLength
    0x01,       // bDescriptorType: device
    0x10,0x01,  // bcdUSB: 1.1
    0x00,       // bDeviceClass
    0x00,       // bDeviceSubClass
    0x00,       // bDeviceProtocoll
    8,          // bMaxPacketSize0
    0xeb,0x03,  // idVendor: Atmel
    0x02,0x00,  // idProduct
    0x00,0x01,  // bcdDevice (1.0)
    0x01,       // iManufacturer
    0x02,       // iProduct
    0x03,       // iSerialNumber
    0x01        // bNumConfigurations
};

static const uint8_t PROGMEM conf_des[] = {
    9,          // bLength
    0x02,       // bDescriptorType: configuration
    0x5e,0x00,  // wTotalLength
    3,          // bNumInterfaces
    0x01,       // bConfigurationValue
    0,          // iConfiguration
    0x80,       // bmAttributes: bus-powered, no remote wakeup
    250,        // MaxPower: 500mA

    /* interface 0: HID keyboard */
    9,          // bLength
    0x04,       // bDescriptorType: interface
    0,          // bInterfaceNumber
    0,          // bAlternateSetting
    2,          // bNumEndpoints
    0x03,       // bInterfaceClass: HID
    0x00,       // bInterfaceSubClass: 0x00
    0x00,       // bInterfaceProtocol: 0x00
    0,          // iInterface: none

    /* HID */
    9,          // bLength
    0x21,       // bDescriptorType: HID
    0x10, 0x01, // bcdHID: 1.1
    0,          // bCountryCode: none
    0x01,       // bNumDescriptors
    0x22,       // bDescriptorType: report
    0x3b, 0x00, // wDescriptorLength (report)

    /* ep 1 */
    7,          // bLength
    0x05,       // bDescriptorType: Endpoint
    0x81,       // bEndpointAddress: IN
    0x03,       // bmAttributes: Interrupt
    64, 0,      // wMaxPacketSize
    1,          // bInterval: [ms]

    /* ep 2 */
    7,          // bLength
    0x05,       // bDescriptorType: Endpoint
    0x02,       // bEndpointAddress: OUT
    0x03,       // bmAttributes: Interrupt
    64, 0,      // wMaxPacketSize
    1,          // bInterval: [ms]

    /* interface 1: ACM control */
    9,          // bLength
    0x04,       // bDescriptorType: Interface
    1,          // bInterfaceNumber
    0,          // bAlternateSetting
    1,          // bNumEndpoints FIXME: do we need a notification endpoint here?
    0x02,       // bInterfaceClass: CDC
    0x02,       // bInterfaceSubClass: ACM
    0x01,       // bInterfaceProtocol: AT command protocol FIXME: would 0xff USB_CDC_ACM_PROTO_VENDOR be more appropriate here?
    0,          // iInterface: none

    /* recycle ep 1 here */
    7,          // bLength
    0x05,       // bDescriptorType: Endpoint
    0x81,       // bEndpointAddress: IN
    0x03,       // bmAttributes: Interrupt
    64, 0,      // wMaxPacketSize
    1,          // bInterval: [ms]

    /* CDC header */
    0x05,       // bFunctionLength
    0x24,       // bDescriptorType: CS_INTERFACE
    0x00,       // bDescriptorSubtype: header
    0x10,0x01,  // bcdCDC: 1.1

    /* CDC ACM */
    0x04,       // bFunctionLength
    0x24,       // bDescriptorType: CS_INTERFACE
    0x02,       // bDescriptorSubtype: ACM
    0x00,       // bmCapabilities: Support nothing, this is an emulated serial, anyway.

    /* union */
    0x05,       // bFunctionLength
    0x24,       // bDescriptorType: CS_INTERFACE
    0x06,       // bDescriptorSubtype: union
    1,          // bControlInterface: control interface number
    2,          // bSubordinateInterface0: data interface number

    /* call management */
    /* not needed, I think
    0x05,       // bFunctionLength
    0x24,       // bDescriptorType: CS_INTERFACE
    0x01,       // bDescriptorSubtype: call management
    0x02,       // bmCapabilites: in-band, off-device call management over data interface
    2,          // bDataInterface: call management data interface number
    */

    /* interface 2: ACM data */
    9,          // bLength
    0x04,       // bDescriptorType: Interface
    2,          // bInterfaceNumber
    0,          // bAlternateSetting
    2,          // bNumEndpoints
    0x0a,       // bInterfaceClass: CDC data
    0x00,       // bInterfaceSubClass: no data subclass
    0x00,       // bInterfaceProtocol: no data protocol
    0,          // iInterface: none

    /* ep 3 */
    7,          // bLength
    0x05,       // bDescriptorType: Endpoint
    0x83,       // bEndpointAddress: IN
    0x02,       // bmAttributes: Bulk
    64, 0,      // wMaxPacketSize
    0,          // bInterval (ignored here)

    /* ep 4 */
    7,          // bLength = 0x07
    0x05,       // bDescriptorType: Endpoint
    0x04,       // bEndpointAddress: OUT
    0x02,       // bmAttributes: Bulk
    64, 0,      // wMaxPacketSize
    0           // bInterval (ignored here)
};

static const uint8_t PROGMEM lang_des[] = {
    4,         // bLength
    0x03,      // bDescriptorType: String
    0x09, 0x04 // wLANGID[0]: english (USA) (Supported Lang. Code 0)
};

static const uint8_t PROGMEM manu_des[] = {
    18,   // bLength = 0x04, Groesse Deskriptor in Byte
    0x03, // bDescriptorType = 0x03, Konstante String = 3
    'W', 0, 'E', 0, 'I', 0, 'G', 0, 'U', 0, '.', 0, 'L', 0, 'U', 0
};

static const uint8_t PROGMEM prod_des[] = {
    26,   // bLength
    0x03, // bDescriptorType: String
    'U', 0, 'S', 0, 'B', 0, '-', 0, 'K', 0, 'E', 0, 'Y', 0, 'B', 0, 'O', 0, 'A', 0, 'R', 0, 'D', 0
};

static const uint8_t PROGMEM seri_des[] = {
    12,   // bLength
    0x03, // bDescriptorType: string
    'W', 0, 'H', 0, '1', 0, '2', 0, 'c', 0
};

static const uint8_t PROGMEM rep_des[] = {
    0x05, 0x01, // Usage_Page (Generic Desktop)
    0x09, 0x06, // Usage (Keyboard)
    0xA1, 0x01, // Collection (Application)
    0x05, 0x07, // Usage page (Key Codes)
    0x19, 0xE0, // Usage_Minimum (224)
    0x29, 0xE7, // Usage_Maximum (231)
    0x15, 0x00, // Logical_Minimum (0)
    0x25, 0x01, // Logical_Maximum (1)
    0x75, 0x01, // Report_Size (1)
    0x95, 0x08, // Report_Count (8)
    0x81, 0x02, // Input (Data,Var,Abs) = Modifier Byte
    0x81, 0x01, // Input (Constant) = Reserved Byte
    0x19, 0x00, // Usage_Minimum (0)
    0x29, 0x65, // Usage_Maximum (101)
    0x15, 0x00, // Logical_Minimum (0)
    0x25, 0x65, // Logical_Maximum (101)
    0x75, 0x08, // Report_Size (8)
    0x95, 0x06, // Report_Count (6)
    0x81, 0x00, // Input (Data,Array) = Keycode Bytes(6)
    0x05, 0x08, // Usage Page (LEDs)
    0x19, 0x01, // Usage_Minimum (1)
    0x29, 0x05, // Usage_Maximum (3)
    0x15, 0x00, // Logical_Minimum (0)
    0x25, 0x01, // Logical_Maximum (1)
    0x75, 0x01, // Report_Size (1)
    0x95, 0x03, // Report_Count (3)
    0x91, 0x02, // Output (Data,Var,Abs) = LEDs (3 bits)
    0x95, 0x03, // Report_Count (3)
    0x91, 0x01, // Output (Constant) = Pad (3 bits)
    0xC0        // End_Collection
};

void usb_ep0_setup(void) {
    uint8_t bmRequestType;
    uint8_t bRequest;
    uint8_t wValue_l;
    uint8_t wValue_h;
    uint8_t wIndex_l;
    uint8_t wIndex_h;
    uint8_t wLength_l;
    uint8_t wLength_h;

    bmRequestType = UEDATX;
    bRequest      = UEDATX;
    wValue_l      = UEDATX;
    wValue_h      = UEDATX;
    wIndex_l      = UEDATX;
    wIndex_h      = UEDATX;
    wLength_l     = UEDATX;
    wLength_h     = UEDATX;

    UNUSED(bmRequestType); /* properly handling it is not strictly necessary. TODO check this. */

    UEINTX &= ~(1<<RXSTPI); /* transmit ACK */

//    if (wIndex_h) { /* sanity check, we only have less than 256 interfaces. */
//        UECONX |= (1<<STALLRQ);
//        return;
//    }

    switch (bRequest) {
    case 0x00: /* GET_STATUS (3-phase) FIXME is handling of this necessary? */
        UEDATX  = 0;
        UEDATX  = 0;
        UEINTX &= ~(1<<TXINI); /* transmit ACK */
        while (!(UEINTX & (1 << RXOUTI)))
            ; /* wait for ZLP */
        UEINTX &= ~(1<<RXOUTI);
        break;

    case 0x05: /* SET_ADDRESS (2-phase; no data phase) */
        UDADDR  = (wValue_l & 0x7F);
        UEINTX &= ~(1<<TXINI); /* transmit ZLP */
        while (!(UEINTX & (1 << TXINI)))
            ; /* wait for bank release */
        UDADDR |= (1<<ADDEN);
        break;

    case 0x06: /* GET_DESCRIPTOR (3-phase transfer) */
        switch (wValue_h) {
        case 1: /* device */
            usb_send_descriptor(dev_des, sizeof(dev_des));
            break;

        case 2: /* configuration */
            /* Manchmal erfragt Windows unsinnig hohe Werte (groesser als
             * 256 Byte). Dadurch steht dann im Lowbyte ev. ein falscher
             * Wert Es wird hier davon ausgegangen, dass nicht mehr als 256
             * Byte angefragt werden können. */
            if (wLength_h || (wLength_l > sizeof(conf_des)) || !wLength_l)
                wLength_l = sizeof(conf_des);
            usb_send_descriptor(conf_des, wLength_l);
            break;

        case 3: /* string */
            switch (wValue_l) {
                case 0:
                    usb_send_descriptor(lang_des, sizeof(lang_des));
                    break;
                case 1:
                    usb_send_descriptor(prod_des, sizeof(prod_des));
                    break;
                case 2:
                    usb_send_descriptor(manu_des, sizeof(manu_des));
                    break;
                case 3:
                    usb_send_descriptor(seri_des, sizeof(seri_des));
                    break;
            }
            break;

        case 34: /* HID report */
            usb_send_descriptor(rep_des, (wLength_l > sizeof(rep_des)) ? sizeof(rep_des) : wLength_l);
            break;

        default:
            break;
        }
        break;

    case 0x09: /* SET_CONFIGURATION (2-phase; no data phase) */
        for (uint8_t i=4; i; i--) { /* backwards due to memory assignment */
            UENUM    = i;
            UECONX  &= ~(1<<EPEN);
            UECFG1X &= ~(1<<ALLOC);
        }

        usb_init_endpoint(1, 3, 1, 3, 0); /* interrupt in, 64 bytes, 1 bank */
        usb_init_endpoint(2, 3, 0, 3, 0); /* interrupt out, 64 bytes, 1 bank */
        usb_init_endpoint(3, 2, 1, 3, 0); /* bulk in, 64 bytes, 1 bank */
        usb_init_endpoint(4, 2, 0, 3, 0); /* bulk out, 64 bytes, 1 bank */

//            UEIENX |= (1<<RXOUTE);
// FIXME needed?            UEIENX |= (1<<TXINE);

        UENUM   = 0;
        UEINTX &= ~(1<<TXINI); // sende ZLP (Erfolg, löscht EP-Bank)
        while (!(UEINTX & (1 << TXINI)))
            ;
        break;

    case 0x20: /* CDC ACM set line coding */
    default:
        UECONX |= (1<<STALLRQ);
        break;
    }
}

void usb_send_descriptor(const uint8_t *d, uint8_t len) {
    uint8_t oldlen = len;

    while (len) {
        while (!(UEINTX & (1<<TXINI)))
            if (UEINTX & (1<<NAKOUTI))
                break; /* host wants to cancel */

        uint8_t j = (len > 8 ? 8 : len);
        while (j--)
            UEDATX = pgm_read_byte(d++);
        len = (len > 8) ? len-8 : 0;

        if (UEINTX & (1<<NAKOUTI))
            break;

        UEINTX &= ~(1<<TXINI);
    }

    if (oldlen&0x7 == 0 && !(UEINTX & (1<<NAKOUTI))) {
        while (!(UEINTX & (1<<TXINI)))
            ; /* wait */
        UEINTX &= ~(1<<TXINI);
    }

    while (!(UEINTX & (1<<NAKOUTI)))
        ; /* wait for host ack */

    UEINTX &= ~(1<<NAKOUTI);
    UEINTX &= ~(1<<RXOUTI);
}

void usb_ep_in(volatile uint8_t *buf, volatile uint8_t *count) {
    if (!(UEINTX & (1<<NAKINI)))
        return;
    /* host requests data */
    UEINTX &= ~(1<<NAKINI);

    /* bank is free */
    UEINTX &= ~(1<<TXINI);

    uint8_t n = *count;
    for (int i=0; i<n; i++)
        UEDATX = buf[i];
    *count = 0;

    UEINTX &= ~(1<<FIFOCON); /* release fifo */
}

void usb_ep_out(volatile uint8_t *buf, volatile uint8_t *count, uint8_t size) {
    if (!(UEINTX & (1<<RXOUTI))) /* host did not send data */
        return;
    UEINTX &= ~(1<<RXOUTI); /* send ACK */

    uint8_t i = 0;
    while ((i < size) && (UEINTX & (1<<RWAL)))
        buf[i++] = UEDATX;
    *count = i;

    UEINTX &= ~(1<<FIFOCON); /* release fifo */
}
