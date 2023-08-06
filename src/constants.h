#ifndef CONSTANTS_HEADER
#define CONSTANTS_HEADER

// Determine pixel at x, y is black or white.
// grabbed from https://devforum.play.date/t/c-macros-for-working-with-playdate-bitmap-data/7706
#define samplepixel(data, x, y, rowbytes) (((data[(y)*rowbytes+(x)/8] & (1 << (uint8_t)(7 - ((x) % 8)))) != 0) ? kColorWhite : kColorBlack)

#define SCREEN_MID_X LCD_COLUMNS/2.0f
#define SCREEN_MID_Y LCD_ROWS/2.0f

#endif
