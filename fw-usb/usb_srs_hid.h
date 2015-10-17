/*! \file usb_srs_hid_v1_2.h
********************************************************************************
 Headerdatei fuer usb_srs_hid_v1_2.c
 \author weigu www.weigu.lu
******************************************************************************/

/* Copyright (c) 2013   Guy WEILER         weigu[at]weigu[dot]lu

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   Licence: GPL-3        */

#ifndef _USB_SRS_HID_H_
#define _USB_SRS_HID_H_

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

//uncomment if not needed
//falls nicht erwünscht, auskommentieren 
//#define MOUSE        
#define KEYBOARD         

// 4 Makros um die Lesbarkeit zu erhoehen:
#define CBI(adr,bit)    (adr &= ~(1<<bit)) // Loesche Bit in SF-Reg. (ASM)
#define SBI(adr,bit)    (adr |=  (1<<bit)) // Setze Bit in SF-Reg. (ASM)
#define low(x)   ((x) & 0xFF)
#define high(x)  (((x)>>8) & 0xFF)

//Teensy 1.0: AT90USB162  16 MHz
//Teensy 2.0: ATmega32u4  16 MHz
//AT90USBKEY: AT90USB1287  8 MHz

// Den PLL Vorteiler festlegen
#if defined(__AVR_ATmega32U4__)
#if F_CPU == 8000000
#define PLLPRE 0x00
#else
#define PLLPRE 0x10
#endif
#endif
#if defined(__AVR_AT90USB162__) || defined(__AVR_ATmega32U2__)
#if F_CPU == 8000000
#define PLLPRE 0x00
#else
#define PLLPRE 0x04
#endif
#endif
#if defined(__AVR_AT90USB1287__)
#if F_CPU == 8000000
#define PLLPRE 0x0C
#else
#define PLLPRE 0x14
#endif
#endif

volatile uint8_t blinkflag;        // Blinken ein/aus

volatile uint8_t Ep1_buf[64];
volatile uint8_t Ep2_buf[64];
volatile uint8_t Ep3_buf[64];
volatile uint8_t Ep4_buf[64];
                                  // Erstes Byte res. für Anzahl der Datenbytes
volatile uint8_t Ep1_flag;        // Flag Senden zum PC
volatile uint8_t Ep2_flag;        // Flag Empfang vom PC
volatile uint8_t Ep1_cnt;         // Zähler Senden zum PC
volatile uint8_t Ep2_cnt;         // Zähler Empfang vom PC

/*  ty TYPE:        0 = Control, 1 = Isochron, 2 = Bulk 3 = Interrupt
    di DIRECTION:   0 = OUT, 1 = IN (ausser Control)
    si SIZE:        0 = 8 Bytes, 1 = 16 Bytes, 2 = 32 Bytes, 3 = 64 Bytes,
                    4 = 128 Bytes (nur EP1), 5 = 256 Bytes (EP1), andere reserv.
    ba BANK:        0 = 1 Bank, 1 = 2 Baenke, andere res. */

#define Ep0_ty 0    //Control
#define Ep0_di 0    //OUT    
#define Ep0_si 0    //8 Bytes    
#define Ep0_ba 0    //1 Bank    
#define Ep0_fs 8    

#define Ep1_ty 3    //Interrupt
#define Ep1_di 1    //INT
#define Ep1_si 3    //64 Bytes  
#define Ep1_ba 0    //1 Bank
#define Ep1_fs_l 64 
#define Ep1_fs_h 0 

#define Ep2_ty 3    //Interrupt
#define Ep2_di 0    //IN
#define Ep2_si 3    //64 Byte    
#define Ep2_ba 0    //1 Bank
#define Ep2_fs_l 64 
#define Ep2_fs_h 0 

#define Ep3_ty 2    //Bulk
#define Ep3_di 1    //IN
#define Ep3_si 3    //64 Byte    
#define Ep3_ba 0    //1 Bank
#define Ep3_fs_l 64 
#define Ep3_fs_h 0 

#define Ep4_ty 2    //Bulk
#define Ep4_di 0    //OUT
#define Ep4_si 3    //64 Byte    
#define Ep4_ba 0    //1 Bank
#define Ep4_fs_l 64 
#define Ep4_fs_h 0 

// Status codes
#define Lang_i     0   // LanguageDescriptorIndex
#define Manu_i     1   // ManufacturerStringIndex    
#define Prod_i     2   // ProductStringIndex
#define Seri_i     3   // SerialNumberStringIndex

// Basisfunktionen
void usb_init_device(void);
void usb_init_endpoint(uint8_t nu,uint8_t ty,uint8_t di,uint8_t si,uint8_t ba);

// Enumeration
void usb_ep0_setup(void);
void usb_send_descriptor(uint8_t de[] ,uint8_t db);

// Anwendungskommunikation
void usb_ep1_in(void);
void usb_ep2_out(void);

#endif
