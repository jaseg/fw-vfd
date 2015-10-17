/* Licence: GPL-3
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


ISR(USB_GEN_vect) {
    if (UDINT & (1 << EORSTI)) { // End Of ReSeT?
        UDINT &= ~(1<<EORSTI);      // sperre EORSTI
        usb_init_endpoint(0, 0, 0, 0, 0); /* control out, 8 bytes, 1 bank */
        UEIENX |= (1<<RXSTPE);
    }
}

ISR(USB_COM_vect) {
    switch (UEINT) {
    case 1: /* ep 0 */
        UENUM = 0;
        if (UEINTX & (1 << RXSTPI))
            usb_ep0_setup();
        break;

    case 2: /* ep 1 */
        UENUM = 1;
        usb_ep1_in();
        break;

    case 4: /* ep 2 */
        UENUM = 2;
        usb_ep2_out();
        break;

    case 8: /* ep 3 */
        UENUM = 3;
        usb_ep3_in();
        break;

    case 16: /* ep 4 */
        UENUM = 4;
        usb_ep4_out();
        break;
    }
}


void usb_init_device(void) {
#if defined(__AVR_ATmega32U4__)
    UHWCON = (1<<UVREGE); /* enable usb voltage regulator */
#endif
#if defined(__AVR_AT90USB1287__)// Der ATMEL Bootloader nutzt einige Interrupts
    USBCON = (1<<FRZCLK);
    OTGIEN = 0;                 // Deaktiviere diese Interrupts damit sauberer Start
    UDIEN  = 0;
    UHIEN  = 0;
    UEIENX = 0;
    UPIENX = 0;
    UHWCON = (1<<UIMOD);        // Enable Device Modus und Regelkreis zur Versorgung
    UHWCON = (1<<UIMOD) | (1<<UVREGE);                 // der D+ und D- Leitung einschalten
#endif
#if defined(__AVR_AT90USB162__) || defined(__AVR_ATmega32U2__) || defined(__AVR_ATmega8U2__)
    USBCON = (1<<USBE) | (1<<FRZCLK);
#else
    USBCON = (1<<USBE) | (1<<FRZCLK) | (1<<OTGPADE);
#endif
    /* toggle FRZCLK to wake up module */
    USBCON &= ~(1<<FRZCLK);
    USBCON |= (1<<FRZCLK);
    /* start USB PLL */
#if defined(__AVR_ATmega8U2__)
    PLLCSR = 0x04; /* my headers are strange */
#else
    PLLCSR = PLLPRE;
#endif
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
    Manu_i,     // iManufacturer
    Prod_i,     // iProduct
    Seri_i,     // iSerialNumber
    0x01        // bNumConfigurations
};

static const uint8_t PROGMEM conf_des[] = {
    9,          // bLength
    0x02,       // bDescriptorType: configuration
    0x40,0x00,  // wTotalLength
    0x02,       // bNumInterfaces
    0x01,       // bConfigurationValue
    0,          // iConfiguration
    0x80,       // bmAttributes: bus-powered, no remote wakeup
    250,        // MaxPower: 500mA

    /* interface 0 */
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
    64, 0,      // wMaxPacketSize: EP1_FIFO_Size;
    1,          // bInterval: [ms]

    /* ep 2 */
    7,          // bLength
    0x05,       // bDescriptorType: Endpoint
    0x02,       // bEndpointAddress: OUT
    0x03,       // bmAttributes: Interrupt
    64, 0,      // wMaxPacketSize: EP2_FIFO_Size;
    1,          // bInterval: [ms]

    /* interface 1 ***/
    9,          // bLength
    0x04,       // bDescriptorType: Interface
    1,          // bInterfaceNumber
    0,          // bAlternateSetting
    2,          // bNumEndpoints
    0x02,       // bInterfaceClass: CDC
    0x02,       // bInterfaceSubClass: ACM
    0xff,       // bInterfaceProtocol: USB_CDC_ACM_PROTO_VENDOR
    0,          // iInterface: none

    /* ep 3 */
    7,          // bLength
    0x05,       // bDescriptorType: Endpoint
    0x83,       // bEndpointAddress: IN
    0x02,       // bmAttributes: Bulk
    64, 0,      // wMaxPacketSize = EP1_FIFO_Size;
    0,          // bInterval (ignored here)

    /* ep 4 */
    7,          // bLength = 0x07
    0x05,       // bDescriptorType: Endpoint
    0x04,       // bEndpointAddress: OUT
    0x02,       // bmAttributes: Bulk
    64, 0,      // wMaxPacketSize = EP2_FIFO_Size;
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

static const struct {
    const uint8_t *desc;
    uint8_t size;
} descs[NUM_DESCS] = {
    [Lang_i] = {.desc=lang_des, .size=sizeof(lang_des)},
    [Manu_i] = {.desc=manu_des, .size=sizeof(lang_des)},
    [Prod_i] = {.desc=prod_des, .size=sizeof(lang_des)},
    [Seri_i] = {.desc=seri_des, .size=sizeof(lang_des)}
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
    uint8_t des_bytes;

    bmRequestType = UEDATX;
    bRequest      = UEDATX;
    wValue_l      = UEDATX;
    wValue_h      = UEDATX;
    wIndex_l      = UEDATX;
    wIndex_h      = UEDATX;
    wLength_l     = UEDATX;
    wLength_h     = UEDATX;

    UNUSED(bmRequestType); /* properly handling it is not strictly necessary. TODO check this. */
    UNUSED(wIndex_l);
    UNUSED(wIndex_h);

    UEINTX &= ~(1<<RXSTPI); /* transmit ACK */

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
        UDADDR |= (1<<ADDEN); // Adresse aktivieren
        break;

    case 0x06: /* GET_DESCRIPTOR (3-phase transfer) */
        switch (wValue_h) {
        case 1: /* device */
            usb_send_descriptor((uint8_t *)dev_des, sizeof(dev_des));
            break;

        case 2: /* configuration */
            des_bytes = wLength_l;
            /* Manchmal erfragt Windows unsinnig hohe Werte (groesser als
             * 256 Byte). Dadurch steht dann im Lowbyte ev. ein falscher
             * Wert Es wird hier davon ausgegangen, dass nicht mehr als 256
             * Byte angefragt werden können. */
            if (wLength_h || (wLength_l > 0x40) || (wLength_l == 0))
                des_bytes = 0x40;
            usb_send_descriptor((uint8_t *)conf_des, sizeof(conf_des));
            break;

        case 3: /* string */
            if (wValue_l < NUM_DESCS)
                usb_send_descriptor(descs[wValue_l].desc, descs[wValue_l].size);
            break;

        case 34: /* HID report */
            des_bytes = wLength_l;
            usb_send_descriptor((uint8_t *)rep_des, des_bytes);
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

        UEIENX |= (1<<NAKINE);
        UEIENX |= (1<<RXOUTE);

        UENUM   = 0;
        UEINTX &= ~(1<<TXINI); // sende ZLP (Erfolg, löscht EP-Bank)
        while (!(UEINTX & (1 << TXINI)))
            ;
        break;

    default:
        UECONX |= (1<<STALLRQ);
        break;
    }
}

void usb_send_descriptor(const uint8_t *d, uint8_t len) {
    for (uint16_t i=0; i<=len; i++) {
        if (UEINTX & (1<<RXOUTI))
            return; /* host wants to cancel */

        UEDATX = pgm_read_byte(d++);

        if (!(i&7)) {
            UEINTX &= ~(1<<TXINI);
            while (!(UEINTX & ((1<<RXOUTI) | (1<<TXINI))))
                ; /* wait for bank release or host ack */
        }
    }

    if (!(UEINTX & (1<<RXOUTI))) {
        UEINTX &= (1<<TXINI);
        while (!(UEINTX & (1<<RXOUTI)))
            ; /* wait for host ack */
    }

    UEINTX &= ~(1<<RXOUTI);
}

void usb_ep1_in(void) {
    if (UEINTX & (1<<NAKINI)) { /* host requests data */
        UEINTX &= ~(1<<NAKINI);

        if (UEINTX & (1<<TXINI)) {
            UEINTX &= ~(1<<TXINI); /* clear bank */

            uint8_t n = ep1_cnt;
            for (int i=0; i<n; i++)
                UEDATX = ep1_buf[i];
            ep1_cnt = 0;

            UEINTX &= ~(1<<FIFOCON); /* release fifo */
        }
    }
}

void usb_ep2_out(void) {
    if (UEINTX & (1<<RXOUTI)) { /* host sent data */
        UEINTX &= ~(1<<RXOUTI); /* send ACK */
        uint8_t i;
        for (i=0; i<sizeof(ep2_buf); i++) {
            if (UEINTX & (1<<RWAL)) {
                ep2_buf[i++] = UEDATX;
            } else {
                break;
            }
        }
        ep2_cnt = i;
        UEINTX &= ~(1<<FIFOCON); /* release fifo */
    }
}
