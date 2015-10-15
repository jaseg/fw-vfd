/*! \file usb_small_lib_hid_firmware_v1_2_c.c
********************************************************************************
 main source for using the small usb library using hid class
 
 www.weigu.lu/b/usb

 (inspired by the lib from S. Salewski (http://http://www.ssalewski.de)
 
 \author weigu www.weigu.lu

        In der Makefile muss der Chip und die Quarzfrequenz für das entsprechende
        Board festgelegt werden!

        Teensy 2.0 (ATmega32u4, 16 MHz, www.pjrc.com/teensy):

        Eingaenge:   ADC0 (PF0) analog
        Ausgaenge:   PD6 (LED on board) und PB0-PB3 LEDS

        AT90USBKEY (AT90USB1287, 8 MHz, www.atmel.com/tools/AT90USBKEY.aspx):

        Eingaenge:   Temperatursensor an ADC0 (PF0) analog
        Ausgaenge:   PD4-7 (LEDs on board, PD6 blinkt)

        Teensy 1.0 (AT90USB162, 16 MHz, www.pjrc.com/teensy):

        Eingaenge:   PC4
        Ausgaenge:   PD6 (onboard) und PB0-PB3 LEDS

        Informationen zur Funktionsweise:

        Hauptprogramm zur Nutzung der HID-Bibliothek
        Ein Timerinterrupt eine LED an PD6 mit 1 Hz blinken.
        Die gesamte Arbeit zum USB-Bus wird mittels Interrupt-Service-Routinen
        abgewickelt. Alle USB-Routinen und Unterprogramme (wie zum Bsp. "USBINI")
        befinden sich in der Bibliothek "usb_srs_vendor_v1_2.c"

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


#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "lib/usb_srs_hid_v1_2.h"

//Hier kann die USB-Maus bzw. das USB-Keyboard freigeschaltet werden
//#define MOUSE            //falls nicht erwünscht, auskommentieren
//#define KEYBOARD           //falls nicht erwünscht, auskommentieren

/*! \brief  Timer1 Compare Match A Interrupt
    Toggelt LED an PD6 */
ISR (TIMER1_COMPA_vect)
{
  SBI (PIND,6); //Toggelt bei neuen Chips das Pin!  
}

/*! \brief Hauptprogramm
 */
int main(void)
{
  // Wirkung des Fusebit CKDIV ueberschreiben
  CLKPR = (1<<CLKPCE);  
  CLKPR = 0;
  // Timer1 initialisieren, Interrupt ein mal pro Sekunde
  // Vorteiler 1024, Zaehlschritte 7811-1 ergeben bei 16MHz 1 Sekunde
  TCCR1A = 0;
  TCCR1B = 0x0D;           // WGM12 = 1 Teiler = 1024
#if F_CPU == 8000000
  OCR1AH = high(3905);    // Achtung High zuerst!
  OCR1AL = low(3905);
#else
  OCR1AH = high(7811);    // Achtung High zuerst!
  OCR1AL = low(7811);
#endif
  TIMSK1 |= (1 << OCIE1A); // ;OCI1A Interrupt erlauben

  //Eingänge festlegen (! Anpassen je nach Chip)
  DDRB = 0;             // external Pull-Up 20MOhm
  DDRF = 0;             // external Pull-Up 20MOhm  

  // Ausgänge festlegen
  DDRD |= (1<<DDD6); // PD6 = Ausgang

  // USB starten (erlaubt auch Interrupts (sei))
  usb_init_device();    // [usb_srs.c]
  blinkflag = 1;

  void sendkey(uint8_t mod, uint8_t code)
  {
    Ep1_cnt = 3;        //3 bytes
    cli();
    //1 Byte = key modifier Byte (7-0):
    //R GUI, R ALT, R SHIFT, R CTRL,L GUI, L ALT, L SHIFT, L CTRL
    Ep1_buf[0] = mod;   //modifier byte
    Ep1_buf[1] = 0x00;  //reserved byte = 0
    Ep1_buf[2] = code;  //scan code (up to 6 keys)
    sei();
    Ep1_flag = 1;       //set EP1 flag
    _delay_ms(50);       //higher than bInterval in Endpoint Descriptor!
    Ep1_cnt = 3;        //3 bytes
    cli();
    Ep1_buf[0] = 0x00;  //modifier byte
    Ep1_buf[1] = 0x00;  //reserved byte = 0
    Ep1_buf[2] = 0x00;  ////stop pressing key
    sei();
    Ep1_flag = 1;       //set EP1 flag
    _delay_ms(3);       //higher than bInterval in Endpoint Descriptor!
  }        

  while (1)
  {        
/*    if ((PINB & (1<<PB0)) == 0) sendkey(0x00,0x14); //q
    if ((PINB & (1<<PB1)) == 0) sendkey(0x00,0x1A); //w
    if ((PINB & (1<<PB2)) == 0) sendkey(0x00,0x08); //e
    if ((PINB & (1<<PB3)) == 0) sendkey(0x00,0x15); //r
    if ((PINB & (1<<PB4)) == 0) sendkey(0x00,0x17); //t
    if ((PINB & (1<<PB5)) == 0) sendkey(0x00,0x1C); //z
    if ((PINB & (1<<PB6)) == 0) sendkey(0x00,0x18); //u
    if ((PINB & (1<<PB7)) == 0) sendkey(0x00,0x0C); //i*/
    if ((PINF & (1<<PF0)) == 0) sendkey(0x02,0x14); //o
    /*if ((PINF & (1<<PF1)) == 0) sendkey(0x00,0x13); //p
    if ((PINF & (1<<PF4)) == 0) sendkey(0x00,0x10); //m
    if ((PINF & (1<<PF5)) == 0) sendkey(0x00,0x22); //5
    if ((PINF & (1<<PF6)) == 0) sendkey(0x00,0x23); //6
    if ((PINF & (1<<PF7)) == 0) sendkey(0x00,0x24); //7*/
    _delay_ms(100); 
  }  
}  
