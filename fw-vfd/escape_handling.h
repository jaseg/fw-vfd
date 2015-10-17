#ifndef __ESCAPE_HANDLING_H__
#define __ESCAPE_HANDLING_H__

#include <stdint.h>

#define STYLE_BOLD                 1
#define STYLE_UNDERLINE            2
#define STYLE_BLINK_SLOW           4
#define STYLE_BLINK_FAST           8
#define STYLE_STRIKETHROUGH     0x10
#define STYLE_INVERT            0x20
#define STYLE_ERROR             0x80


typedef enum {
    LED_ERROR,
    LED_PROG,
    LED_POWER
} led_t;


uint8_t parse_escape_sequence(char *seq, uint8_t len, uint8_t oldstyle);

void set_led(led_t led, uint8_t val);

#endif /* __ESCAPE_HANDLING_H__ */
