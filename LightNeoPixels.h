// This code is adopted from the following great article:
// https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
#ifndef LNP
#define LNP
#include <Arduino.h>

// NeoPixel pinout
#define PIXEL_PORT  PORTB
#define PIXEL_BIT   0

// NeoPixel timings in nanoseconds
#define T1H  900
#define T1L  600
#define T0H  400
#define T0L  900
#define RES  250000

// Nanoseconds per instruction cycle
#define NS_PER_CYCLE (1000000000L / F_CPU) // 1Gns/s / 8MHz

// Show colors on NPs
void show();

// Sending one bit to NPs
inline void send_bit(bool value);

// Sending one byte to NPs
inline void send_byte(unsigned char p_byte);

// Send one full color (1 pixel) to NPs
inline void send_color(unsigned char r, unsigned char g, unsigned char b);

// Send array of colors to NPs
void send_strip(unsigned char* r, unsigned char* g,  unsigned char* b, unsigned short length);

#endif