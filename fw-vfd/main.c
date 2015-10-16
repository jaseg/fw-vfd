
#include <avr/io.h>
#include <util/delay.h>


#define PULSE_LENGTH_US 100


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


void sel_pulse(uint8_t val) {
    uint8_t v = PORTD;
    PORTD = v | (0x04<<val);
    _delay_us(PULSE_LENGTH_US);
    PORTD = v;
}

void vfd_latch_data(uint8_t data[5]) {
    PORTA  = data[0];
    sel_pulse(0);
    PORTA  = data[1];
    sel_pulse(1);
    PORTA  = data[2];
    sel_pulse(2);
    PORTA  = data[3];
    sel_pulse(3);
    PORTA  = data[4];
    sel_pulse(4);
}

void mux_pulse(uint8_t output) {
    PORTC |= output;
    PORTB |= 0x04;
    _delay_us(PULSE_LENGTH_US);
    PORTB &= 0x04;
    PORTC &= ~output;
}

/* also sets one anode latch enable pin (A7) */
void vfd_grid_reset(void) {
    DDRD  |= 0x40;
    mux_pulse(4);
    PORTD &= ~0x40;
}

void vfd_grid_next(void) {
    mux_pulse(4);
}


int main(void) {
    DDRD  |= 0x80; /* green  "power" led */
    DDRB  |= 0x03; /* red "error" led (PB0), yellow "prog" led (PB1) */

    DDRD  |= 0x02; /* PD0 - RXD, PD1 - TXD */

    DDRA   = 0xff; /* VFD data out */
    DDRD  |= 0x7c; /* VFD anode latch select / A3-A7 */
    DDRB  |= 0x04; /* mux enable, active low */
    PORTB &= ~0x04;
    DDRC  |= 0x07; /* mux address line out / A0-A2 */

    uint16_t ubrr_val = F_CPU/16/(BAUDRATE-1);
    UBRRH  = ubrr_val>>8;
    UBRRL  = ubrr_val&0xff;
    UCSRB = (1<<TXEN);
    UCSRC = (1<<URSEL) | (3<<UCSZ0);

    while (23) {
        set_led(0, 1);
        uart_putc('a');
        _delay_ms(500);
        set_led(1, 1);
        uart_putc('b');
        _delay_ms(500);
        set_led(2, 1);
        uart_putc('c');
        _delay_ms(500);
        set_led(0, 0);
        uart_putc('d');
        _delay_ms(500);
        set_led(1, 0);
        uart_putc('e');
        _delay_ms(500);
        set_led(2, 0);
        uart_putc('f');
        uart_putc('\n');
        _delay_ms(500);
        continue;
        vfd_grid_reset();
        uint8_t data[5] = {0, 0, 0, 0, 0};
        for (uint8_t i=0; i<5; i++) {
            for (uint8_t j=1; j; j<<=1) {
                data[i] |= j;
                vfd_latch_data(data);
                vfd_grid_next();
                _delay_ms(10);
            }
        }
    }
}
