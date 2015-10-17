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

#include <stdint.h>
#include <avr/pgmspace.h> 
#include <avr/interrupt.h>
#include "usb_srs_hid.h"

ISR(USB_GEN_vect)
{
  if (UDINT & (1<<EORSTI)) { // End Of ReSeT?
    CBI (UDINT,EORSTI); // sperre EORSTI
    usb_init_endpoint(0,Ep0_ty, Ep0_di, Ep0_si, Ep0_ba);    
    SBI(UEIENX,RXSTPE);  
   }
}

ISR(USB_COM_vect)
{
  switch (UEINT) {
    //EP0: wenn Setup-Paket von PC angekommen, dieses behandeln (Enumeration)
    case 1: /* ep 1 */
        UENUM = 0;
        if (UEINTX & (1<<RXSTPI))
            usb_ep0_setup();
        break;
    case 2:
        UENUM = 1;
        usb_ep1_in();
        break;
    case 4: /* ep 2 */
        UENUM = 2;
        usb_ep2_out();
        break;
    case 8:  /* ep 3 */
        usb_ep3_in();
    case 16: /* ep 4 */
        usb_ep4_out();
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
  USBCON = ((1<<USBE) | (1<<FRZCLK)); // USB Sender/Empf. und Takt einschalten
#else
  USBCON = ((1<<USBE) | (1<<FRZCLK) | (1<<OTGPADE));// USB Sender/Empf.,
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
  UDIEN = (1<<EORSTE);  // erlaube End Of ReSeT Interrupt

  memset(Ep1_buf, 0, sizeof(Ep1_buf));
  memset(Ep2_buf, 0, sizeof(Ep1_buf));
  memset(Ep3_buf, 0, sizeof(Ep1_buf));
  memset(Ep4_buf, 0, sizeof(Ep1_buf));

  Ep1_flag = 0;
  Ep2_flag = 0;
  Ep1_cnt = 0;
  Ep2_cnt = 0;

  sei();
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

  /*** Device Descriptor ***/
  static const uint8_t PROGMEM dev_des[] = {
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
    0x01      // bNumConfigurations = 1, Anzahl Konfigurationen*/
  };     
  /*** Configurations Descriptors ***/
  static const uint8_t PROGMEM conf_des[] =  {
    9,        //bLength
    0x02,     //bDescriptorType: configuration
    0x40,0x00,//wTotalLength
    0x02,     //bNumInterfaces
    0x01,     //bConfigurationValue
    0,        //iConfiguration
    0x80,     //bmAttributes: bus-powered, no remote wakeup
    250,      //MaxPower: 500mA
    /*** Interface 1 Descriptor ***/
    9,        //bLength
    0x04,     //bDescriptorType: interface
    0,        //bInterfaceNumber
    0,        //bAlternateSetting
    2,        //bNumEndpoints
    0x03,     //bInterfaceClass: HID
    0x00,     //bInterfaceSubClass: 0x00
    0x00,     //bInterfaceProtocol: 0x00
    0,        //iInterface: none
    /*** HID Descriptor ***/
    9,        //bLength
    0x21,     //bDescriptorType: HID
    0x10,0x01,//bcdHID: 1.1
    0,        //bCountryCode: none
    0x01,     //bNumDescriptors
    0x22,     //bDescriptorType: report
    0x3b,0x00,//wDescriptorLength (report)
    /*** EP1 Descriptor ***/
    7,        //bLength
    0x05,     //bDescriptorType: Endpoint
    0x81,     //bEndpointAddress: IN
    0x03,     //bmAttributes: Interrupt
    Ep1_fs_l,Ep1_fs_h,    //wMaxPacketSize: EP1_FIFO_Size;
    1,        //bInterval: [ms]
    /*** EP2 Descriptor ***/
    7,        //bLength
    0x05,     //bDescriptorType: Endpoint
    0x02,     //bEndpointAddress: OUT
    0x03,     //bmAttributes: Interrupt
    Ep2_fs_l,Ep2_fs_h,    //wMaxPacketSize: EP2_FIFO_Size;
    1,        //bInterval: [ms]
    /*** Interface 2 Descriptor ***/
    9,        //bLength
    0x04,     //bDescriptorType: Interface
    1,        //bInterfaceNumber
    0,        //bAlternateSetting
    2,        //bNumEndpoints
    0x02,     //bInterfaceClass: CDC
    0x02,     //bInterfaceSubClass: ACM
    0xff,     //bInterfaceProtocol: USB_CDC_ACM_PROTO_VENDOR
    0,        //iInterface: none
    /*** EP1 Descriptor ***/
    7,        //bLength
    0x05,     //bDescriptorType: Endpoint
    0x83,     //bEndpointAddress: IN
    0x02,     //bmAttributes: Bulk
    Ep3_fs_l,Ep3_fs_h,    //wMaxPacketSize = EP1_FIFO_Size;
    0,        //bInterval (ignored here)
    /*** EP2 Descriptor ***/
    7,        //bLength = 0x07
    0x05,     //bDescriptorType: Endpoint
    0x04,     //bEndpointAddress: OUT
    0x02,     //bmAttributes: Bulk
    Ep4_fs_l,Ep4_fs_h,    //wMaxPacketSize = EP2_FIFO_Size;
    0         //bInterval (ignored here)
  };                    
  /*** Language Descriptor ***/      
  static const uint8_t PROGMEM lang_des[] = {
    4,        //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
    0x09,0x04 //wLANGID[0] = 0x0409 = English USA (Supported Lang. Code 0) 
  };
  /*** Manufacturer Descriptor ***/          
  static const uint8_t PROGMEM manu_des[] = {
    18,       //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
              //bString = Unicode Encoded String (16 Bit!)
    'W',0,'E',0,'I',0,'G',0,'U',0,'.',0,'L',0,'U',0 
  };        
  /*** Product Descriptor ***/       
  static const uint8_t PROGMEM prod_des[] = {
    26,       //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
              //bString = Unicode Encoded String (16 Bit!)
    'U',0,'S',0,'B',0,'-',0,'K',0,'E',0,'Y',0,'B',0,'O',0,'A',0,'R',0,'D',0
  };        
  /*** Serial Descriptor ***/      
  static const uint8_t PROGMEM seri_des[] = {
    12,       //bLength = 0x04, Groesse Deskriptor in Byte
    0x03,     //bDescriptorType = 0x03, Konstante String = 3
              //bString = Unicode Encoded String (16 Bit!) 
    'W',0,'H',0,'1',0,'2',0,'c',0
  };        
  /*** Report Descriptor ***/
  static const uint8_t PROGMEM rep_des[] = {
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
    0x29,0x05,  // Usage_Maximum (3)
    0x15,0x00,  // Logical_Minimum (0)
    0x25,0x01,  // Logical_Maximum (1)
    0x75,0x01,  // Report_Size (1)
    0x95,0x03,  // Report_Count (3)
    0x91,0x02,  // Output (Data,Var,Abs) = LEDs (3 bits)
    0x95,0x03,  // Report_Count (3)
    0x91,0x01,  // Output (Constant) = Pad (3 bits)
    0xC0 // End_Collection
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
  if ((bmRequestType & 0x60) == 0 || (bmRequestType & 0x60) == 1) {                                 
    switch (bRequest) {
      case 0x00: //GET_STATUS 3 Phasen (optional kann geloescht werden!)
        UEDATX = 0;
        UEDATX = 0;
        CBI(UEINTX, TXINI); //Daten absenden (ACK) und FIFO löschen
        while (!(UEINTX & (1 << RXOUTI)));  //Warten bis ZLP vom PC
        CBI(UEINTX, RXOUTI);//Flag loeschen
        break;

      case 0x05: //SET_ADDRESS 2 Phasen (keine Datenphase)
        UDADDR = (wValue_l & 0x7F); // Speichere Adresse UADD (ADDEN = 0)
        CBI(UEINTX, TXINI); //IN Paket (ZLP) senden, EP Bank loeschen 22.12 S272 
        while (!(UEINTX & (1<<TXINI)));// Warte bis fertig (EP Bank wieder frei)
        SBI(UDADDR, ADDEN); // Adresse aktivieren
        break;

      case 0x06: //GET_DESCRIPTOR 3 Phasen Transfer 
          switch (wValue_h)
          {
            case 1: // Device-Descriptor          
              usb_send_descriptor((uint8_t*) dev_des, sizeof(dev_des));         
              break;
            case 2: // Configuration-Descriptor
              des_bytes = wLength_l;
              //Manchmal erfragt Windows unsinnig hohe Werte (groesser als 256
              //Byte). Dadurch steht dann im Lowbyte ev. ein falscher Wert
              //Es wird hier davon ausgegangen, dass nicht mehr als 256 Byte 
              //angefragt werden können.
              if (wLength_h || (wLength_l > 0x40) || (wLength_l == 0)) 
                 des_bytes = 0x40;
              usb_send_descriptor((uint8_t*) conf_des, sizeof(conf_des));
              break;
            case 3: // String-Descriptor
              switch (wValue_l)
              {
                case Lang_i:
                  usb_send_descriptor((uint8_t*) lang_des, sizeof(lang_des));
                  break;
                case Manu_i:
                  usb_send_descriptor((uint8_t*) manu_des, sizeof(lang_des));
                  break;
                case Prod_i:
                  usb_send_descriptor((uint8_t*) prod_des, sizeof(lang_des));
                  break;
                case Seri_i:
                  usb_send_descriptor((uint8_t*) seri_des, sizeof(lang_des));
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
        break;

      case 0x09: //SET_CONFIGURATION 2 Phasen (keine Datenphase)
        for (uint8_t i=Nr_eps;i==1;i--) { /* backwards due to memory assignment */
          UENUM = i; // Waehle EP i  
          CBI(UECONX, EPEN); // Schalte Endpunkt ab
          CBI(UECFG1X, ALLOC); // Gib den Speicher frei
        }

        usb_init_endpoint(1,Ep1_ty, Ep1_di, Ep1_si, Ep1_ba);
        usb_init_endpoint(2,Ep2_ty, Ep2_di, Ep2_si, Ep2_ba);
        usb_init_endpoint(3,Ep3_ty, Ep3_di, Ep3_si, Ep3_ba);
        usb_init_endpoint(4,Ep4_ty, Ep4_di, Ep4_si, Ep4_ba);

        SBI(UEIENX, NAKINE);  //NAK INterrupts für IN Endpunkte erlauben
        SBI(UEIENX, RXOUTE);  //RX OUT Interrupts für OUT Endpunkte erlauben
 
        UENUM = 0; // Waehle EP0  
        CBI(UEINTX, TXINI); //sende ZLP (Erfolg, löscht EP-Bank)
        while (!(UEINTX & (1<<TXINI)));
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
  for (uint16_t i=1;i<=db;i++) {
    if (UEINTX & (1 << RXOUTI))
        return; // PC will abbrechen!

    UEDATX = pgm_read_byte(&de[i-1]); // Schreibe Byte ins FIFO
    //immer nach 8 Bytes das Paket absenden und Speicherbank löschen
    if ((i % Ep0_fs) == 0) {
      CBI(UEINTX,TXINI); //IN Paket (FIFO) senden, EP Bank loeschen 22.12 S272 
      while (!(UEINTX & ((1<<RXOUTI) | (1<<TXINI)))); // Warte bis Bank frei oder ACK Host
    }
  }

  if (!(UEINTX & (1 << RXOUTI))) {
      CBI(UEINTX,TXINI); //IN Paket (ZLP) senden, EP Bank loeschen 22.12
      while (!(UEINTX & (1 << RXOUTI))); //Warten bis ACK(ZLP) vom PC angekommen
  }

  CBI(UEINTX, RXOUTI);// Handshake um Interrupt zu bestätigen  
}


/*! \brief Endpunktbehandlung EP1 (IN)*/
void usb_ep1_in(void)	//Endpunkt 1 IN
{ 
  if (UEINTX & (1 << NAKINI)) { // Sind Daten vom PC angefragt worden?
    CBI(UEINTX, NAKINI); //NAKIN Interrupt Flag loeschen

    if (UEINTX & (1 << TXINI)) { 
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
  if (UEINTX & (1 << RXOUTI)) { //Daten vom PC angekommen?
    CBI(UEINTX, RXOUTI); // Datenpaket bestaetigen (ACK)
    for (int i = 0; i < 64; i++) {
      if (UEINTX & (1 << RWAL)) { //Daten im FIFO?
        Ep2_buf[i] = UEDATX;   // Lese USB FIFO
        Ep2_cnt=i+1;
      } else {
          break;
      }
    }
    Ep2_flag = 1;
    CBI(UEINTX, FIFOCON); //FIFO wieder freigeben
  }
}  
