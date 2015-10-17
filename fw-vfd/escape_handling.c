
#include "escape_handling.h"

uint8_t parse_escape_sequence(char *seq, uint8_t len, uint8_t oldstyle) {
    if (len == 0)
        return STYLE_ERROR;

    if (seq[0] != '[')
        return STYLE_ERROR;

    if (seq[len-1] != 'm') {
        if (len != 3)
            return STYLE_ERROR;

        if (seq[1] < '0' || seq[1] > '2')
            return STYLE_ERROR;

        if (seq[2] == 'E') /* used to set LEDs */
            set_led(seq[1] - '0', 1);
        else if (seq[2] == 'U') /* used to unset LEDs */
            set_led(seq[1] - '0', 0);
        else
            return STYLE_ERROR;

        return oldstyle;

    }

    seq[len-1] = '\0';
    seq++;

    for (; seq[0]; seq++) {
        if (seq[1] && seq[1] != ';') {
            if (seq[2] != ';') /* value too large, skip ahead */
                while (*seq && *seq != ';')
                    seq++;

            if (seq[0] == '2')
                seq++;
            else
                return STYLE_ERROR;
        }

        char mod = 0;
        switch(seq[0]){
            case '0': /* reset style */
                oldstyle &= ~0x3f;
                break;
            case '1': /* bold */
                mod = STYLE_BOLD;
                break;
            case '4': /* underline */
                mod = STYLE_UNDERLINE;
                break;
            case '5': /* slow blink */
                mod = STYLE_BLINK_SLOW;
                break;
            case '6': /* rapid blink */
                mod = STYLE_BLINK_FAST;
                break;
            case '7': /* color invert on */
                mod = STYLE_INVERT;
                break;
            case '9': /* strike-through */
                mod = STYLE_STRIKETHROUGH;
                break;
            default:
                /* valid, unsupported escape sequence */
                break;
        }

        if (seq[-1] == '2') {
            oldstyle &= ~mod;
        } else {
            oldstyle |= mod;
        }
    }
    return oldstyle;
}
