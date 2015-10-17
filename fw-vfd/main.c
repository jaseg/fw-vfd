
#include <avr/io.h>
#include <util/delay.h>


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

void vfd_latch_data(uint8_t data[5]) {
    PORTA  = data[0];
    grid_strobe(0);
    PORTA  = data[1];
    grid_strobe(1);
    PORTA  = data[2];
    grid_strobe(2);
    PORTA  = data[3];
    grid_strobe(3);
    PORTA  = data[4];
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
    UCSRB = (1<<TXEN);
    UCSRC = (1<<URSEL) | (3<<UCSZ0);

    while (23) {

        set_led(LED_ERROR, 1);

        vfd_grid_reset();
        uint8_t data[5] = {0, 0, 0, 0, 0};
        for (uint8_t i=0; i<5; i++) {
            for (uint8_t j=1; j; j<<=1) {
                data[i] |= j;
                vfd_latch_data(data);
                vfd_grid_next();
                _delay_us(250);
            }
            if (i==2)
                set_led(LED_ERROR, 0);
        }
    }
}
