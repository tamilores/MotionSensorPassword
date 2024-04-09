// This code is adopted from the following great article:
// https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
#include <LightNeoPixels.h>
#include <Arduino.h>

// Show colors on NPs
void show() {
    _delay_us((RES / 1000UL) + 1);  // Round up since the delay must be _at_least_ this long (too short might not work, too long not a problem)
}

// Sending one bit to NPs
inline void send_bit(bool value) {
    if (value) { // 0 bit
        asm volatile (
            "sbi %[port], %[bit] \n\t" // Set the output bit
            ".rept %[onCycles] \n\t"   // Execute NOPs to delay exactly the specified number of cycles
            "nop \n\t"
            ".endr \n\t"
            "cbi %[port], %[bit] \n\t" // Clear the output bit
            ".rept %[offCycles] \n\t"  // Execute NOPs to delay exactly the specified number of cycles
            "nop \n\t"
            ".endr \n\t" ::
            [port]      "I"(_SFR_IO_ADDR(PIXEL_PORT)),
            [bit]       "I"(PIXEL_BIT),
            [onCycles]  "I"(T1H / NS_PER_CYCLE - 2), // 1-bit width less overhead  for the actual bit setting, note that this delay could be longer and everything would still work
            [offCycles] "I"(T1L / NS_PER_CYCLE - 2) // Minimum interbit delay. Note that we probably don't need this at all since the loop overhead will be enough, but here for correctness
        );
    } else { // 1 bit
        // **************************************************************************
        // This line is really the only tight goldilocks timing in the whole program!
        // **************************************************************************
        asm volatile (
            "sbi %[port], %[bit] \n\t" // Set the output bit
            ".rept %[onCycles] \n\t"   // Now timing actually matters. The 0-bit must be long enough to be detected but not too long or it will be a 1-bit
            "nop \n\t"                 // Execute NOPs to delay exactly the specified number of cycles
            ".endr \n\t"
            "cbi %[port], %[bit] \n\t" // Clear the output bit
            ".rept %[offCycles] \n\t"  // Execute NOPs to delay exactly the specified number of cycles
            "nop \n\t"
            ".endr \n\t" ::
            [port]      "I"(_SFR_IO_ADDR(PIXEL_PORT)),
            [bit]       "I"(PIXEL_BIT),
            [onCycles]  "I"(T0H / NS_PER_CYCLE - 2),
            [offCycles] "I"(T0L / NS_PER_CYCLE - 2)
        );
    }
}

// Sending one byte to NPs
inline void send_byte(unsigned char p_byte) {
    unsigned char byte = 0;
    // Flip bit order according to NeoPixels expectation
    for (unsigned char b = 0; b < 8; b++) {
        byte |= (p_byte & 0x01);
        byte <<= 1;
        p_byte >>= 1;
    }
    // Send byte
    for (unsigned char b = 0; b < 8; b++) {
        send_bit(byte & 0x01);
        byte >>= 1;
    }
} 

// Send one full color (1 pixel) to NPs
inline void send_color(
    unsigned char r,
    unsigned char g,
    unsigned char b
) {
    send_byte(g);
    send_byte(r);
    send_byte(b);
}

// Send array of colors to NPs
void send_strip(
    unsigned char* r, unsigned char* g,  unsigned char* b,
    unsigned short length
) {
    cli();
    for (unsigned short i = 0; i < length; i++)
        send_color(r[i], g[i], b[i]);
    sei();
    show();
}