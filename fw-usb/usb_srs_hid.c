/*! \file usb_srs.c
********************************************************************************
 small usb library for AVRs (mega32u4, mega32u2,at90usb162, at90usb1287)
 using hid class
 2 endpoints (1*IN 1*OUT) with 64 Byte
 
 www.weigu.lu/b/usb

 (inspired by the lib from S. Salewski (http://http://www.ssalewski.de)
 
 \author weigu www.weigu.lu

 
        Titel:  Kleine USB-Bibliothek:
                Unterprogramme und Interruptroutinen zur Nutzung von USB mit
                ATMEL-AVR-ICs.
                HID Klasse

        Informationen zur Beschaltung:
      
        Controller: AT90USB162, AT90USB1287, ATmega32u4, ATmega32u2
                    (siehe Zuweisungen makefile)
        Quarz:      8 oder 16 MHz  (siehe Zuweisungen makefile)
                
        Informationen zur Funktionsweise:
      
        Die gesamte Arbeit zum USB-Bus wird mittels Interrupt-Service-Routinen
        abgewickelt. Alle dazu benötigten USB-Routinen und Unterprogramme
        befinden sich in dieser USB-Bibliothek.
        
        Im Hauptprogramm muss nur die Bibliothek eingebunden werden:
        #include "lib/usb_srs_hid_v1_2.h"
        und das Unterprogramm usb_init_device() aufgerufen werden.
        
        Fuer beide Endpunkte steht ein 64-Byte großer Buffer zur Verfuegung. 

        Beim Endpunkt 1 (IN) fragt der PC Daten an.
        Durch Setzen eines Flags (EP1_FLAG) im Hauptprogramm wird dem Endpunkt
        mitgeteilt, wenn neue Daten vorliegen. Die Anzahl der vorhandenen Bytes
        steht dann in der Variablen EP1_CNT. Die Daten selbst werden im Buffer
        (EP1_BUF) abgelegt.
        
        Ueber den Endpunkt 2 (OUT) sendet der PC Daten.
        Die Hauptschleife im Hauptprogramm erkennt an einem Flag (EP2_FLAG)
        wenn Daten vorliegen.
        Die Daten liegen im Buffer (EP2_BUF). Die Variable EP2_CNT gibt an wie 
        viele Bytes gesendet wurden.
        
        Das Freischalten der Maus-Emulation bzw. der Tastaturemulation
        erfolgt in der Headerdatei lib/usb_srs_hid_v1_2.h
      
        Weitere Dokumentation auf www.weigu.lu/b/usb


******************************************************************************/

/* Copyright (c) 2009   Guy WEILER         weigu[at]weigu[dot]lu

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

#include <stdint.h>
#include <avr/pgmspace.h> 
#include <avr/interrupt.h>
#include "usb_srs_hid_v1_2.h"

/*! \brief USB General Interrupt (S253)*/ 
ISR(USB_GEN_vect)
{
  if (UDINT & (1<<EORSTI)) // End Of ReSeT?
  {
    CBI (UDINT,EORSTI); // sperre EORSTI
    // Initialisiere EP0
    usb_init_endpoint(0,Ep0_ty, Ep0_di, Ep0_si, Ep0_ba);    
    SBI(UEIENX,RXSTPE);  
   }
}

/*! \brief Endpoint/Pipe Communication Interrupt Service Routine (S254) */    
ISR(USB_COM_vect)
{
   // EP Nummer durch Bitposition in UEINT gekennzeichnet 2^EP = 1
  switch (UEINT)
  {
    //EP0: wenn Setup-Paket von PC angekommen, dieses behandeln (Enumeration)
    case 1: UENUM = 0; if (UEINTX & (1<<RXSTPI)) usb_ep0_setup(); break;
    //EP1: IN
    case 2: UENUM = 1; usb_ep1_in(); break;
    //Ep2: OUT
    case 4: UENUM = 2; usb_ep2_out(); break;
    
    //// hier erweitern 
    //case 8: //UENUM = 3;  ///      break;
    
    default: break;
  }
}

/*! \brief USB-Aktivierung (Full-Speed 12Mbit/s) und Interrupts erlauben */
void usb_init_device(void)
{
#if defined(__AVR_ATmega32U4__)
  UHWCON = (1<<UVREGE); //Regelkreis zur Versorgung der D+ und D- Leitung einschalten
#endif        
#if defined(__AVR_AT90USB1287__) // Der ATMEL Bootloader nutzt einige Interrupts
  USBCON = (1<<FRZCLK);   
  OTGIEN = 0;                   // Deaktiviere diese Interrupts damit sauberer Start
  UDIEN  = 0;
  UHIEN  = 0;
  UEIENX = 0;
  UPIENX = 0;
  UHWCON = (1<<UIMOD);          //Enable Device Modus und Regelkreis zur Versorgung
  UHWCON = ((1<<UIMOD) | (1<<UVREGE)); // der D+ und D- Leitung einschalten
#endif
#if defined(__AVR_AT90USB162__) || defined(__AVR_ATmega32U2__)
  USBCON = ((1<<USBE) | (1<<FRZCLK)); // 0xA0 ;USB Sender/Empf. und Takt einschalten
#else
  USBCON = ((1<<USBE) | (1<<FRZCLK) | (1<<OTGPADE)); //0xB0 USB Sender/Empf.,
                                                    // Takt und VBUS-Leitung einschalten
#endif  
  USBCON &= ~(1<<FRZCLK); // FRZCLK=0, noetig, da sonst kein WAKEUP Interrupt
  USBCON |= (1<<FRZCLK);  // FRZCLK=1 Strom sparen
  // Starte PLL
  PLLCSR = PLLPRE;      // PLL Vorteiler fuer den Quarz (siehe Zuweisungen)
  PLLCSR |= (1<<PLLE);  // starte PLL (PLLEnable=1)
  while (!(PLLCSR &(1<<PLOCK)));     // Warte bis PLOCK = 1 (PLL eingerastet)
  USBCON &= ~(1<<FRZCLK); // FRZCLK=0, aktiviere den Takt
  UDCON = 0; // Attach: Verbinde das Device
  UDIEN = (1<<EORSTE);  //0x08 erlaube End Of ReSeT Interrupt

  // Endpunkt Buffer, Flags und Zaehler loeschen
  for (int i = 0; i < 64; i++)
  {
    Ep2_buf[i] = 0x00;
    Ep1_buf[i] = 0x00;
  }
  Ep1_flag = 0;
  Ep2_flag = 0;
  Ep1_cnt = 0;
  Ep2_cnt = 0;

  sei();     // Interrupts global erlauben
}

/*! \brief Aktiviere Endpunkt
    \PARAM usb_init_endpoint()
    nu EP_Number: 0...6
    ty Type:      0 = Control, 1 = Isochron, 2 = Bulk 3 = Interrupt
    di Direction: 0 = OUT, 1 = IN (ausser Control)
    si Size:      0 = 8 Bytes, 1 = 16 Bytes, 2 = 32 Bytes, 3 = 64 Bytes, 
                 4 = 128 Bytes (nur EP1), 5 = 256 Bytes (EP1), andere reserviert
    ba Bank:      0 = 1 Bank, 1 = 2 Baenke, andere res.
    Flussdiagramm zum Aktivieren eines EP S269 hier keine Fehlerkontrolle
    */
void usb_init_endpoint(uint8_t nu,uint8_t ty,uint8_t di,uint8_t si,uint8_t ba)
{
  UENUM = nu; // Waehle Endpoint 
  SBI(UECONX,EPEN); // Aktiviere EP
  UECFG0X = ((ty << 6) | (di)); // TYPE + DIR festlegen
  UECFG1X = ((si << 4) | (ba << 2)); // SIZE + BanK festlegen
  SBI(UECFG1X,ALLOC); // Speicher zuweisen    
}

/*! \brief 
    Wird aufgerufen, wenn ein Received SETUP Interrupt (RXSTPI in UEINTX)
    auftritt (EP0 S272).
    */ 
void usb_ep0_setup(void)
{
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint8_t wValue_l;
  uint8_t wValue_h;
  uint8_t wIndex_l;
  uint8_t wIndex_h;
  uint8_t wLength_l;
  uint8_t wLength_h;
  uint8_t des_bytes;

  // Ab hier folgen die Deskriptortabellen im Flash 
  /*** Device Descriptor ***/
  static const uint8_t PROGMEM dev_des[] = 
  {
    18,       // bLength = 18 (0x12), Groesse des Deskriptors in Byte
    0x01,     // bDescriptorType = 0x01, Konstante Device = 1
    0x10,0x01,// bcdUSB = 0x0110, USB_Spec1_1
    0x00,     // bDeviceClass = 0x00, Klassencode: 
    0x00,     // bDeviceSubClass = 0x00, Unterklassencode
    0x00,     // bDeviceProtocoll = 0x00, Protokollcode 
    Ep0_fs,   // bMaxPacketSize0 = EP0FS, max. Paketgroesse EP0 (hier 8 B)
    0xeb,0x03,// idVendor = 0x03eb, Atmel Code durch usb.org vergeben
    0x02,0x00,// idProduct = 0x0002, Produkt ID (weigu: 2 fuer HID)
    0x00,0x01,// bcdDevice = 0x0100, Release Nummer Geraet
    Manu_i,   // iManufacturer = Index fuer String-Deskriptor Hersteller
    Prod_i,   // iProduct = Index fuer String-Deskriptor    Produkt
    Seri_i,   // iSerialNumber = Index fuer String-Deskriptor Seriennummer
    0x01      // bNumConfigurations = 1, Anzahl moeglicher Konfigurationen*/
  };     
  /*** Configurations Descriptor ***/
  static const uint8_t PROGMEM conf_des[] =  
  {
    9,        //bLength = 0x09, Groesse Deskriptor in Byte
    0x02,     //bDescriptorType = 0x02, Konstante Configuration = 2
    low(wTotalLength),high(wTotalLength),  //wTotalLength, Laenge Configuration
              //Deskriptor + alle untergeordnete Deskriptoren.
    0x01,     //bNumInterfaces = 1
    0x01,     //bConfigurationValue = 1, darf nicht Null sein (sonst geht
              //Geraet in den nicht-konfigurierten Zustand))
    0,        //iConfiguration = 0, Index fuer Str.-Deskr. Conf. (0=kein Txt)
    0x80,     //bmAttributes = 0x80,bus-powered, kein remote wakeup Bit 7=1
    250,      //MaxPower = 250(dezimal), in 2mA Schritten! (500mA)
    /*** Interface Descriptor ***/
    9,        //bLength = 0x09, Groesse Deskriptor in Byte
    0x04,     //bDescriptorType = 0x04, Konstante Interface = 4
    0,        //bInterfaceNumber = 0;
    0,        //bAlternateSetting = 0;
    Nr_eps,   //bNumEndpoints = USB_Endpoints;
    0x03,     //bInterfaceClass = 0x03 HID, Klassencode:
    0x00,     //bInterfaceSubClass = 0x00, Unterklassencode
    0x00,     //bInterfaceProtocol = 0x00, Protokollcode
    0,        //iInterface = 0, Index fuer String-Deskr. Interf. (0=kein Txt)
    /*** HID Descriptor ***/
    9,        //bLength = 0x09, Groesse Deskriptor in Byte
    0x21,     //bDescriptorType = 0x21 (HID-Klasse)
    0x10,0x01,//bcdHID = 0x0110, HID_Spec 1.1
    0,        //bCountryCode = 0 (wird hier nicht unterstuetzt)
    0x01,     //bNumDescriptors = 1 Anzahl der untergeordneten Deskriptoren
    0x22,     //bDescriptorType = 0x22, Report Descriptor (obligatorisch)
    low(HidDesLength),high(HidDesLength),   //'wDescriptorLength Laenge des Report Deskriptor 
    /*** EP1 Descriptor ***/
    7,        //bLength = 0x07, Groesse Deskriptor in Byte
    0x05,     //bDescriptorType = 0x05, Konstante Endpoint = 5
    0x81,     //bEndpointAddress = 0x81, Bit 7 = 1 (IN) Bit 0-3 EP-Nummer
    0x03,     //bmAttributes = 0x03, Transfertyp = Interrupt (c=0, iso=1, bulk=2)
    Ep1_fs_l,Ep1_fs_h,    //wMaxPacketSize = EP1_FIFO_Size;
    1,        //bInterval = 1 bei Interrupt: Latenzzeit in Millisekunden
    /*** EP2 Descriptor ***/
    7,        //bLength = 0x07, Groesse Deskriptor in Byte
    0x05,     //bDescriptorType = 0x05, Konstante Endpoint = 5
    0x02,     //bEndpointAddress = 0x82, Bit 7 = 1 (IN) Bit 0-3 EP-Nummer
    0x03,     //bmAttributes = 0x03, Transfertyp = Interrupt (c=0, iso=1, bulk=2)
    Ep2_fs_l,Ep2_fs_h,    //wMaxPacketSize = EP2_FIFO_Size;
    1         //bInterval = 1 bei Interrupt: Latenzzeit in Millisekunden
  };                    
  /*** Language Descriptor ***/      
  static const uint8_t PROGMEM lang_des[] = 
  {
    4,        //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
    0x09,0x04 //wLANGID[0] = 0x0409 = English USA (Supported Lang. Code 0) 
  };
  /*** Manufacturer Descriptor ***/          
  static const uint8_t PROGMEM manu_des[] = 
  {
    18,       //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
              //bString = Unicode Encoded String (16 Bit!)
    'W',0,'E',0,'I',0,'G',0,'U',0,'.',0,'L',0,'U',0 
  };        
  /*** Product Descriptor ***/       
  static const uint8_t PROGMEM prod_des[] = 
  {
#if defined MOUSE
    20,       //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
              //bString = Unicode Encoded String (16 Bit!)
    'U',0,'S',0,'B',0,'-',0,'M',0,'O',0,'U',0,'S',0,'E',0
#elif defined KEYBOARD
    26,       //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
              //bString = Unicode Encoded String (16 Bit!)
    'U',0,'S',0,'B',0,'-',0,'K',0,'E',0,'Y',0,'B',0,'O',0,'A',0,'R',0,'D',0
#else
    22,       //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
              //bString = Unicode Encoded String (16 Bit!)
    'H',0,'I',0,'D',0,'-',0,'D',0,'E',0,'V',0,'I',0,'C',0,'E',0
#endif
  };        
  /*** Serial Descriptor ***/      
  static const uint8_t PROGMEM seri_des[] = 
  {
    12,       //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
              //bString = Unicode Encoded String (16 Bit!) 
    'W',0,'H',0,'1',0,'2',0,'c',0
  };        
  /*** Report Descriptor ***/
  static const uint8_t PROGMEM rep_des[] =
  {
#if defined MOUSE
    // Report Descriptor Mouse     
    0x05,0x01,  //USAGE_PAGE (Generic Desktop) allgemeine Funktion des Geraetes
    0x09,0x02,  //USAGE (Mouse) Funktion des Berichts in der Usage Page
    0xA1,0x01,  //COLLECTION (Application) Beginn einer Gruppe von Einträgen
    0x09,0x01,  //USAGE (Pointer)
    0xA1,0x00,  //COLLECTION (Physical) Gruppe von Einträgen
    0x05,0x09,  //USAGE_PAGE (Buttons) Maustasten
    0x19,0x01,  //USAGE_MINIMUM (Button 1) mindestens eine
    0x29,0x03,  //USAGE_MAXIMUM (Button 3) maximal 3
    0x15,0x00,  //LOGICAL_MINIMUM (0) Legen Bereich für die Werte fest die
    0x25,0x01,  //LOGICAL_MAXIMUM (1) im Bericht enthalten sind (binär)
    0x75,0x01,  //REPORT_SIZE (1) Anzahl der Bits pro Datenelement 1Bit
    0x95,0x03,  //REPORT_COUNT (3) Anzahl der Datenelemente im Bericht 3
    0x81,0x02,  //INPUT (Data,Var,Abs) INPUT Bericht, variable Daten
    0x75,0x05,  //REPORT_SIZE (5) 5 Bits       (füllt Byte?)
    0x95,0x01,  //REPORT_COUNT (1) 1*
    0x81,0x01,  //INPUT (Cnst,Var,Abs) Konstantes Feld
    0x05,0x01,  //USAGE_PAGE (Generic Desktop) Mauszeiger
    0x09,0x30,  //USAGE (X) 48
    0x09,0x31,  //USAGE (Y) 49
    0x09,0x38,  //USAGE (Wheel) 56
    0x15,0x81,  //LOGICAL_MINIMUM (-127) -127 bis 127
    0x25,0x7F,  //LOGICAL_MAXIMUM (127)
    0x75,0x08,  //REPORT_SIZE (8) 8 Bits
    0x95,0x03,  //REPORT_COUNT (3) 3*8 Bits (X,Y,Wheel)
    0x81,0x06,  //INPUT (Data,Var,Rel)
    0xC0,       //END_COLLECTION
    0xC0        //END_COLLECTION
#elif defined KEYBOARD
    // Report Descriptor Keyboard     
    0x05,0x01,  // Usage_Page (Generic Desktop)
    0x09,0x06,  // Usage (Keyboard)
    0xA1,0x01,  // Collection (Application)
    0x05,0x07,  // Usage page (Key Codes)
    0x19,0xE0,  // Usage_Minimum (224)
    0x29,0xE7,  // Usage_Maximum (231)
    0x15,0x00,  // Logical_Minimum (0)
    0x25,0x01,  // Logical_Maximum (1)
    0x75,0x01,  // Report_Size (1)
    0x95,0x08,  // Report_Count (8)
    0x81,0x02,  // Input (Data,Var,Abs) = Modifier Byte
    0x81,0x01,  // Input (Constant) = Reserved Byte
    0x19,0x00,  // Usage_Minimum (0)
    0x29,0x65,  // Usage_Maximum (101)
    0x15,0x00,  // Logical_Minimum (0)
    0x25,0x65,  // Logical_Maximum (101)
    0x75,0x08,  // Report_Size (8)
    0x95,0x06,  // Report_Count (6)
    0x81,0x00,  // Input (Data,Array) = Keycode Bytes(6)
    0x05,0x08,  // Usage Page (LEDs)
    0x19,0x01,  // Usage_Minimum (1)
    0x29,0x05,  // Usage_Maximum (5)
    0x15,0x00,  // Logical_Minimum (0)
    0x25,0x01,  // Logical_Maximum (1)
    0x75,0x01,  // Report_Size (1)
    0x95,0x05,  // Report_Count (5)
    0x91,0x02,  // Output (Data,Var,Abs) = LEDs (5 bits)
    0x95,0x03,  // Report_Count (3)
    0x91,0x01,  // Output (Constant) = Pad (3 bits)
    0xC0 // End_Collection
#else   
    // Report Descriptor HID Device
    0x06,0xA0,0xFF,// USAGE PAGE = 0xFF00 (Vendor Defined Page 1)
    0x09,0x01,     // USAGE (Vendor Usage 1)
                   // Funktion des Berichts in der Usage Page
    0xA1,0x01,     // COLLECTION (Application)
                   // Beginn einer Gruppe von Eintraegen
    0x09,0x03,     // USAGE (Vendor Usage 1)
    0x15,0x00,     // LOGICAL_MINIMUM (1) Legen Bereich Werte fest, die
    0x25,0x40,     // LOGICAL_MAXIMUM (64) im Bericht enthalten sind (bin.)
    0x95,0x02,     // REPORT_COUNT (64) Anzahl der Datenelemente im Bericht
    0x75,0x08,     // REPORT_SIZE (8) Anzahl der Bits pro Datenelement 8Bit
                   // Make sixty-four 8-bit fields (the next time the
                   // parser hits an "Input", "Output", or "Feature" item)
    0x81,0x02,     // INPUT (Data,Var,Abs) INPUT Bericht
                   // Instantiates input packet fields based on the above
                   // report size, count, logical min/max, and usage.
    0x09,0x04,     // USAGE (Vendor Usage 1)
    0x15,0x00,     // LOGICAL_MINIMUM (1) Legen Bereich Werte fest, die
    0x25,0x40,     // LOGICAL_MAXIMUM (64) im Bericht enthalten sind (bin.)
    0x95,0x02,     // REPORT_COUNT (64) Anzahl der Datenelemente im Bericht
    0x75,0x08,     // REPORT_SIZE (8) Anzahl der Bits pro Datenelement 8Bit
    0x91,0x02,     // OUTPUT (Data,Var,Abs) OUTPUT Bericht
                   // Instantiates output packet fields.  Uses same report
                   // size and count as "Input" fields, since nothing
                   // new/different was specified to the parser since the
                   // "Input" item.
    0xC0        // END_COLLECTION
#endif
  };
  
  /* hier erweitern falls weitere EPs benoetigt werden*/
  
  bmRequestType = UEDATX; 
  bRequest = UEDATX; 
  wValue_l = UEDATX; 
  wValue_h = UEDATX; 
  wIndex_l = UEDATX; 
  wIndex_h = UEDATX;
  wLength_l = UEDATX; 
  wLength_h = UEDATX; 
  
  CBI(UEINTX, RXSTPI); //'ACK SU-Paket (ISR abschliessen und FIFO löschen)
  // Type = Standard Device Request? (nur Device oder Interface)
  if ((bmRequestType & 0x60) == 0 || (bmRequestType & 0x60) == 1)
  {                                 
    switch (bRequest)
    {
      case 0x00: //GET_STATUS 3 Phasen (optional kann geloescht werden!)
        UEDATX = 0;
        UEDATX = 0;
        CBI(UEINTX,TXINI); //Daten absenden (ACK) und FIFO löschen
        while (!(UEINTX & (1 << RXOUTI)));  //Warten bis ZLP vom PC
        CBI(UEINTX, RXOUTI);//Flag loeschen
        break;
      case 0x05: //SET_ADDRESS 2 Phasen (keine Datenphase)
        UDADDR = (wValue_l & 0x7F); // Speichere Adresse UADD (ADDEN = 0)
        CBI(UEINTX,TXINI); //IN Paket (ZLP) senden, EP Bank loeschen 22.12 S272 
        while (!(UEINTX & (1<<TXINI)));// Warte bis fertig (EP Bank wieder frei)
        SBI(UDADDR, ADDEN); // Adresse aktivieren
        break;
      case 0x06: //GET_DESCRIPTOR 3 Phasen Transfer 
        {
          switch (wValue_h)
          {
            case 1: // Device-Descriptor          
              des_bytes = pgm_read_byte(&dev_des[0]);
              usb_send_descriptor((uint8_t*) dev_des,des_bytes);         
              break;
            case 2: // Configuration-Descriptor
              des_bytes = wLength_l;
              //Manchmal erfragt Windows unsinnig hohe Werte (groesser als 256
              //Byte). Dadurch steht dann im Lowbyte ev. ein falscher Wert
              //Es wird hier davon ausgegangen, dass nicht mehr als 256 Byte 
              //angefragt werden können.
              if (wLength_h || (wLength_l > wTotalLength) || (wLength_l == 0)) 
                 des_bytes = wTotalLength;
              usb_send_descriptor((uint8_t*) conf_des,des_bytes);
              break;
            case 3: // String-Descriptor
              switch (wValue_l)
              {
                case Lang_i:
                  des_bytes = pgm_read_byte(&lang_des[0]);
                  usb_send_descriptor((uint8_t*) lang_des,des_bytes);
                  break;
                case Manu_i:
                  des_bytes = pgm_read_byte(&manu_des[0]);
                  usb_send_descriptor((uint8_t*) manu_des,des_bytes);
                  break;
                case Prod_i:
                  des_bytes = pgm_read_byte(&prod_des[0]);
                  usb_send_descriptor((uint8_t*) prod_des,des_bytes);
                  break;
                case Seri_i:
                  des_bytes = pgm_read_byte(&seri_des[0]);
                  usb_send_descriptor((uint8_t*) seri_des,des_bytes);
                  break;              
                default: break;
              }              
              break;
            case 34: // Report Deskriptor (0x22)
              des_bytes = wLength_l;
              usb_send_descriptor((uint8_t*) rep_des,des_bytes);
              break;
            default: break; 
          }      
          /*if (!(UEINTX & (1 << RXOUTI))) // es sind noch restliche Bytes abzuschicken
          {
            CBI(UEINTX,TXINI); //IN Paket (ZLP) senden, EP Bank loeschen 22.12
            while (!(UEINTX & (1 << RXOUTI))); //Warten bis ACK(ZLP) vom PC angekommen
          }   
          CBI(UEINTX, RXOUTI);// Handshake um Interrupt zu bestätigen */
        }
        break;
      case 0x09: //SET_CONFIGURATION 2 Phasen (keine Datenphase)
        for (uint8_t i=Nr_eps;i==1;i--) //Benutzte Endpoints abschalten
        {                              //Rückwärts (siehe Speicherbelegung)
          UENUM = i; // Waehle EP i  
          CBI(UECONX, EPEN); // Schalte Endpunkt ab
          CBI(UECFG1X, ALLOC); // Gib den Speicher frei
        }
        //Initialisierung Endpunkt1  (IN-EP)
        usb_init_endpoint(1,Ep1_ty, Ep1_di, Ep1_si, Ep1_ba);
        SBI(UEIENX, NAKINE);  //NAK INterrupts für IN Endpunkte erlauben
        //Initialisierung Endpunkt 2 (OUT-EP)
        usb_init_endpoint(2,Ep2_ty, Ep2_di, Ep2_si, Ep2_ba);
        SBI(UEIENX, RXOUTE);  //RX OUT Interrupts für OUT Endpunkte erlauben
        ////hier erweitern
 
        UENUM = 0; // Waehle EP0  
        CBI(UEINTX, TXINI); //sende ZLP (Erfolg, löscht EP-Bank)
        while (!(UEINTX & (1<<TXINI)));// Warte bis fertig (EP Bank wieder frei)
        break;
      default: SBI(UECONX,STALLRQ); break;
    }
  }
  else SBI(UECONX,STALLRQ); // Kein Standard Request, erfrage STALL Antwort   
}

/*! \brief Sende Deskriptor zum PC (22.14 IN-EP management S 275)
    Es werden nur so viele Bytes gesendet wie angefragt. Falls PC in dieser 
    Phase (bei Control Transaktion) abbrechen moechte, so sendet er ein ZLP 
    Paket (2.14.1.1 S 276). 
    Fuelle FIFO (gegebenfalls mehrmals) und sende Daten.
     */
void usb_send_descriptor(uint8_t de[] ,uint8_t db)
{
  for (uint16_t i=1;i<=db;i++) //Achtung Überlauf bei uint8_t!!
  {
    if (UEINTX & (1 << RXOUTI)) return;// PC will abbrechen!
    UEDATX = pgm_read_byte(&de[i-1]); // Schreibe Byte ins FIFO
    //immer nach 8 Bytes das Paket absenden und Speicherbank löschen
    if ((i % Ep0_fs) == 0) // falls FIFO gefuellt (8 Byte)
    {
      CBI(UEINTX,TXINI); //IN Paket (FIFO) senden, EP Bank loeschen 22.12 S272 
      while (!(UEINTX & ((1<<RXOUTI) | (1<<TXINI)))); // Warte bis Bank frei oder ACK Host
    }
  }
  if (!(UEINTX & (1 << RXOUTI))) // es sind noch restliche Bytes abzuschicken
  {
      CBI(UEINTX,TXINI); //IN Paket (ZLP) senden, EP Bank loeschen 22.12
      while (!(UEINTX & (1 << RXOUTI))); //Warten bis ACK(ZLP) vom PC angekommen
  }   
  CBI(UEINTX, RXOUTI);// Handshake um Interrupt zu bestätigen  
}


/*! \brief Endpunktbehandlung EP1 (IN)*/
void usb_ep1_in(void)	//Endpunkt 1 IN
{ 
  if (UEINTX & (1 << NAKINI)) // Sind Daten vom PC angefragt worden?
  {
    CBI(UEINTX, NAKINI); //NAKIN Interrupt Flag loeschen
    if (UEINTX & (1 << TXINI))   
    { 
      CBI(UEINTX, TXINI);  //loescht EP-Bank
      if (Ep1_flag == 1) 
        for (int i = 0; i < Ep1_cnt; i++) UEDATX = Ep1_buf[i];
      Ep1_flag = 0;
      Ep1_cnt = 0;
      CBI(UEINTX, FIFOCON); //FIFO wieder freigeben
    }
  }
}

/*! \brief Endpunktbehandlung EP2 (OUT)*/
void usb_ep2_out(void) //Endpunkt 2 OUT
{
  if (UEINTX & (1 << RXOUTI)) //Daten vom PC angekommen?
  {
    CBI(UEINTX, RXOUTI); // Datenpaket bestaetigen (ACK)
    for (int i = 0; i < 64; i++)
    {
      if (UEINTX & (1 << RWAL)) //Daten im FIFO?
      {
        Ep2_buf[i] = UEDATX;   // Lese USB FIFO
        Ep2_cnt=i+1;
      }
      else break;
    }
    Ep2_flag = 1;
    CBI(UEINTX, FIFOCON); //FIFO wieder freigeben
  }
}  