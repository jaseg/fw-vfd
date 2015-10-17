
#include <avr/io.h>
#include <util/delay.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <string.h>

#include "escape_handling.h"


extern const uint8_t font[102*5] PROGMEM;


#define PULSE_LENGTH_US 10


typedef enum {
    LED_ERROR,
    LED_PROG,
    LED_POWER
} led_t;

void set_led(led_t led, uint8_t val) {
    switch (led) {
        case LED_ERROR:
            PORTB &= ~1;
            PORTB |= val ? 1 : 0;
            break;
        case LED_PROG:
            PORTB &= ~2;
            PORTB |= val ? 2 : 0;
            break;
        case LED_POWER:
            PORTD &= ~0x80;
            PORTD |= val ? 0x80 : 0;
            break;
    }
}


char to_hex(uint8_t nibble) {
    if (nibble < 0xa)
        return '0'+nibble;
    else
        return 'a'+(nibble-0xa);
}

void uart_putc(char c) {
    while (!(UCSRA & (1<<UDRE)));
    UDR = c;
}


void mux_pulse(uint8_t output) {
    PORTC |= output;
    PORTB &= ~0x04;
    _delay_us(PULSE_LENGTH_US);
    PORTB |= 0x04;
    PORTC &= ~output;
}

void sel_pulse(uint8_t idx) {
    uint8_t v = PORTD;
    PORTD = v | (0x04<<idx);
    _delay_us(PULSE_LENGTH_US);
    PORTD = v;
}

void grid_strobe(uint8_t idx) {
    uint8_t v = PORTD;
    PORTD = v | (0x04<<idx);
    mux_pulse(6);
    PORTD = v;
}

uint8_t blink_counter = 0;

void vfd_latch_data(const uint8_t *data PROGMEM, uint8_t style) {
    char v;
    char inv = !!(style & STYLE_INVERT);
    if (((style & STYLE_BLINK_SLOW) && (blink_counter&0x40))
    || (!(style & STYLE_BLINK_SLOW) && (style & STYLE_BLINK_FAST) && (blink_counter&0x10))) {
        PORTA = 0;
        grid_strobe(0);
        grid_strobe(1);
        grid_strobe(2);
        grid_strobe(3);
        grid_strobe(4);
        mux_pulse(7);
        return;
    }

    v = pgm_read_byte(data+0);
    PORTA  = inv ? ~v : v;
    grid_strobe(0);

    v = pgm_read_byte(data+1);
    PORTA  = inv ? ~v : v;
    grid_strobe(1);

    v = pgm_read_byte(data+2) | (style & STYLE_STRIKETHROUGH ? 0x67 : 0x00);
    PORTA  = inv ? ~v : v;
    grid_strobe(2);

    v = pgm_read_byte(data+3) | (style & STYLE_UNDERLINE ? 0x80 : 0x00);
    PORTA  = inv ? ~v : v;
    grid_strobe(3);

    v = pgm_read_byte(data+4);
    PORTA  = inv ? ~v : v;
    grid_strobe(4);

    mux_pulse(7);
}

/* also sets one anode latch enable pin (A7) */
void vfd_grid_reset(void) {
    PORTD  |= 0x04;
    mux_pulse(4);
    PORTD &= ~0x04;
}

void vfd_grid_next(void) {
    mux_pulse(4);
}


/*              |.........#.........#.........#.........#| 40 chars */
char text[41] = "VFD firmware v0.1 initialized           ";
unsigned char rxpos = 0;
unsigned char escape_rxpos = 0;
char escape_buf[41];
uint8_t style[41] = {{0}};
uint8_t current_style = {0};
unsigned char state;

#define STATE_ESCAPED       0x01

ISR (USART_RXC_vect) {
    char ch = UDR;
    if (state & STATE_ESCAPED) {
        if (escape_rxpos < sizeof(escape_buf)-1)
            escape_buf[escape_rxpos++] = ch;
        if ((ch > '9' && ch != ';' && ch != '[') || ch < '0') {
            escape_buf[escape_rxpos] = '\0';
            uint8_t rv = parse_escape_sequence(escape_buf, escape_rxpos, current_style);
            if (!(rv & STYLE_ERROR))
                current_style = rv;
            state &= ~STATE_ESCAPED;
            escape_rxpos = 0;
        }
    } else {
        if (ch == '\r') {
            rxpos = 0;
            memset(text, ' ', sizeof(text));
            memset(style, 0, sizeof(text));
        } else if (ch == '\n') {
            /* ignore */
        } else if (ch == '\e') {
            state |= STATE_ESCAPED;
            escape_rxpos = 0;
        } else {
            if (rxpos < sizeof(text)) {
                text[rxpos] = ch;
                style[rxpos] = current_style;
                rxpos++;
            }
        }
    }
}


int main(void) {
    DDRD  |= 0x80; /* green  "power" led */
    DDRB  |= 0x03; /* red "error" led (PB0), yellow "prog" led (PB1) */
    set_led(LED_POWER, 1);

    DDRD  |= 0x02; /* PD0 - RXD, PD1 - TXD */

    DDRA   = 0xff; /* VFD data out */
    DDRD  |= 0x7c; /* VFD anode latch select / A3-A7 */
    DDRB  |= 0x04; /* mux enable, active low */
    PORTB |= 0x04;
    DDRC  |= 0x07; /* mux address line out / A0-A2 */

    uint16_t ubrr_val = F_CPU/16/(BAUDRATE-1);
    UBRRH  = ubrr_val>>8;
    UBRRL  = ubrr_val&0xff;
    UCSRB = (1<<TXEN) | (1<<RXEN) | (1<<RXCIE);
    UCSRC = (1<<URSEL) | (3<<UCSZ0);

    sei();

    while (23) {

        set_led(LED_ERROR, 1);

        vfd_grid_reset();
        for (uint8_t i=0; i<40; i++) {
            uint8_t ch = text[39-i];
            vfd_latch_data(font+(ch-32)*5, style[39-i]);
            if (style[i] & STYLE_BOLD)
                _delay_us(1000);
            else
                _delay_us(300);
            vfd_grid_next();
        }

        blink_counter++;
    }
}
