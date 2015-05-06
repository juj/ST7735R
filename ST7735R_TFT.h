// A high performance TFT drawing driver library for Arduino Uno / Atmega 328P to interface
// with the Arduino.cc provided ST7735R TFT display.
// Assumes that wiring is done according to default instructions for Uno at http://www.arduino.cc/en/Guide/TFTtoBoards
// Datasheet available at http://www.adafruit.com/datasheets/ST7735R_V0.2.pdf)
// Only supports hardware SPI mode with 8MHz SPI clock.
// Written by Jukka Jylänki.
// This code is released to public domain.
#pragma once

#include <SPI.h>

// If you want to enable the support for .bmp and .565 drawing functions, #include <SD.h> and #define ST7735R_ENABLE_SDCARD before
// including this file. This will enable the following functions:
//   void ST7735R_Draw565(int x, int y, uint32_t imageDataBlock);
//   void ST7735R_DrawBMP(int x, int y, const BMPImage &image);
//   BMPImage LoadBMPImage(uint32_t fileBlock);
// To convert image files to the .565 file format, see the python script image_to_rgb565.py inside this repository.

// Comment out to omit the definitions of symbols in the header. Control this manually if including this file
// from multiple compilation units.
#define ST7735R_INCLUDE_IMPLEMENTATION

// If you want to enable the text drawing functionality, #define ST7735R_MONACO_FONT before including this file.
// This will enable the following function:
//   void ST7735R_DrawText(int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b, uint8_t bgR, uint8_t bgG, uint8_t bgB);
// which renders text using the Monaco font. To convert custom font files to C headers for inclusion, hack the
// python script ttffont_to_cppheader.py in this repository to suit your purpose.

// For highest performance, instead of using digitalWrite() or a combination of
// portOutputRegister+digitalPinToPort+digitalPinToBitMask to flip bits on I/O pins,
// directly access the memory-mapped port registers to make these one clock cycle
// operations. This requires that we choose and configure at *compile-time*, which
// pin we connect, and also tell the compile which port address and bit in that
// address we are accessing. An excellent diagram to figuring out which PORT+BIT
// a given I/O PIN maps to on Arduino Uno can be found here: http://pighixxx.com/unov3pdf.pdf
// If you are using something else than Uno, find a similar diagram to your board.

// Chip Select pin. If this is low, the TFT chip is active. (LCD CS pin at http://www.arduino.cc/en/Guide/TFTtoBoards)
// (CSX pin on the MCU interface at page 13 of http://www.adafruit.com/datasheets/ST7735R_V0.2.pdf)
// Configure these:
#define ST7735R_CS_PIN 10
#define ST7735R_CS_PORT PORTB
#define ST7735R_CS_BIT (1 << PB2)

// Display Data/Command Mode Select pin. If this is high, we are sending a data byte, and if this is low,
// we are sending a command byte. (D/C pin at http://www.arduino.cc/en/Guide/TFTtoBoards)
// ("D/CX (SCL)" pin on the MCU interface at page 13 of http://www.adafruit.com/datasheets/ST7735R_V0.2.pdf)
// Configure these:
#define ST7735R_RS_PIN 9
#define ST7735R_RS_PORT PORTB
#define ST7735R_RS_BIT (1 << PB1)

// Reset pin. If this is taken low, performs a hard reset on the chip. This is optional to connect.
// Configure this:
#define ST7735R_RST_PIN 8

// Activates the TFT for communication. This is expected to compile down to a single
// assembly instruction like "cbi  0x05, 1" (Clear Bit in I/O register) so should take
// only one clock cycle.
#define BEGIN_TFT() (ST7735R_CS_PORT &= ~ST7735R_CS_BIT)

// The TFT has two separate modes of interpreting the received data. One toggles
// between these modes by flipping a bit. The usual transmission flow proceeds
// by first activating the "command mode", then sending a byte that represents
// the command, after which communication is transitioned to "data mode" to 
// push the data bytes related to that command. Before calling either of these,
// the SPI send line must be free (no pending WRITE_SPI_NOWAIT messages on the
// line, but WAIT_SPI has been called). Both of these should compile down to a single
// instruction like "cbi  0x05, 1" (Clear Bit in I/O register) and "sbi 0x05, 1"
// (Set Bit in I/O register) and should take only one clock cycle each.
#define CHOOSE_COMMAND_MODE (ST7735R_RS_PORT &= ~ST7735R_RS_BIT)
#define CHOOSE_DATA_MODE (ST7735R_RS_PORT |= ST7735R_RS_BIT)

// Deactivates the TFT for communication. Before calling this, the SPI send line
// must be free (no pending WRITE_SPI_NOWAIT messages on the line, but WAIT_SPI
// has been called). One assembly instruction sbi, like above.
#define END_TFT() (ST7735R_CS_PORT |= ST7735R_CS_BIT)

// Pushes a byte to the SPI line, but does not stall to wait until
// the communication has finished. Before calling this function, the line must
// be free for transmission, so after calling this, one must call WAIT_SPI
// at some point to ensure that the line is free to send the next byte.
// This should be a single instruction like "out  0x2e, r21".
#define WRITE_SPI_NOWAIT(b) (SPDR = (b))

// Waits until the SPI line is ready (other end has received the byte we sent
// last). After calling this, it is safe to write to SPI again.
// This while loop is expected to compile down to the following (most efficient?)
// form:
//    in r0, 0x2d (read the input port from the I/O address of the SPSR register)
//    sbrs r0, 7  (Skip executing the next instruction if bit 7 is high (sbrs = Skip if Bit in Register Set)
//    rjmp .-6    (Relative Jump back to the 'in' command if bit 7 was low)
// The 'in' instruction takes 1 clock cycle, 'rjmp' takes two, and 'sbrs' takes
// 1 - 3, depending on the result of the test (predicts zero, i.e. no skip)
// Therefore one iteration of WAIT_SPI takes a total of four cycles in the case
// when the iteration is not done but we jump back.
#define WAIT_SPI do { while (!(SPSR & 0x80)) ; } while(0)

// Does one cycle of no operation on the CPU.
#define NOP asm volatile("nop")

// The function WRITE_SPI_SYNC() writes a byte to SPI, and immediately
// (synchronously) waits that the transmission has safely finished. This is
// like WRITE_SPI_NOWAIT, but safe (and slower), since it ensures that the line
// is free afterwards. The reason that a NOP is inserted is to exploit "phase"
// with respect to the time it takes for the ST7735R to process a SPI byte (in
// relation to the 16MHz Atmega 328P at least). Since WAIT_SPI will always take
// a multiple of 4 clock cycles (as one test is 4 clocks), it means that the
// SPSR register is polled only once every 4 clocks, so depending on when the
// SPI operation finishes and which part the loop is at, there can be a delay
// of 0-3 clocks until we see the SPSR be free. Therefore we can carefully
// "prime" the phase of the loop to ensure that we should always hit the 0
// wasted clocks, which by experimental profiling happens when we insert exactly
// one NOP before entering the loop. Profiling also indicates that a SPI write
// operation takes 33 clocks almost always, so WRITE_SPI_SYNC() is therefore
// considerably much slower than WRITE_SPI_NOWAIT() which only takes one clock.
// The general strategy with all draw operations is to interleave the asynchronous
// WRITE_SPI_NOWAIT() operations with as much of other processing as possible,
// so that we can hide these 33 clock cycle busy loops and do meaningful work
// in that time.
// To experimentally confirm the above, try benchmarking the time that draw
// operations take when inserting 0-5 nops here. The results that were received
// at the time of writing, when drawing a full screen of random pixels:
//    0 nops: 553.8 msecs
//    1 nop:  526.8 msecs
//    2 nops: 535.8 msecs
//    3 nops: 545.0 msecs
//    4 nops: 553.8 msecs
//    5 nops: 526.8 msecs
// and as expected, the performance varies in a multiple of four nops, with the
// phase of 4*k+1 nops being the optimally primed pause.
#define WRITE_SPI_SYNC(b) \
  do { \
  WRITE_SPI_NOWAIT(b); \
  NOP; \
  WAIT_SPI; \
  } while(0)

#ifdef ST7735R_ENABLE_SDCARD
#include "SDCardBlockRead.h"
#else
#define BEGIN_SDCARD() ((void)0)
#define END_SDCARD() ((void)0)
#endif

// Transitions to command mode, sends the given command, and transitions back to
// data mode. The SPI line should be free before calling this. After this function
// finishes, the SPI line is free for data transfer.
#define SEND_COMMAND(cmd) \
  do { \
    CHOOSE_COMMAND_MODE; \
    WRITE_SPI_SYNC(cmd); \
    CHOOSE_DATA_MODE; \
  } while(0)

// For high performance operation, we can't afford to do very small transactions inside
// the individual operations. Therefore if you are using transactions, wrap all draw
// operations inside a block of form
//
// ST7735R_BEGIN_TRANSACTION();
//    draw commands;
// ST7735R_END_TRANSACTION();
//
#if defined(SPI_HAS_TRANSACTION)
#define ST7735R_BEGIN_TRANSACTION() SPI.beginTransaction(SPISettings(16000000, MSBFIRST, SPI_MODE0))
#define ST7735R_END_TRANSACTION() SPI.endTransaction()
#else
#define ST7735R_BEGIN_TRANSACTION() ((void)0)
#define ST7735R_END_TRANSACTION() ((void)0)
#endif

#define ST7735R_WIDTH  160
#define ST7735R_HEIGHT 128

// The MCU command interface implements the following instructions. These
// are called with the SEND_COMMAND(cmd) instruction.
#define ST7735R_NOP     0x00 // No operation
#define ST7735R_SWRESET 0x01 // Sofware reset
#define ST7735R_RDDID   0x04 // Read Display ID (four reads follows)
#define ST7735R_RDDST   0x09 // Read Display Status (five reads follows)
#define ST7735R_RDDPM   0x0A // Read Display Power (two reads follows)
#define ST7735R_RDDMADCTL 0x0B // Read Display (two reads follows)
#define ST7735R_RDDCOLMOD 0x0C // Read Display Pixel (two reads follows)
#define ST7735R_RDDIM   0x0D // Read Display Image (two reads follows)
#define ST7735R_RDDSM   0x0E // Read Display Signal (two reads follows)
#define ST7735R_SLPIN   0x10 // Sleep in & booster off. Default after reset: "Sleep In"
#define ST7735R_SLPOUT  0x11 // Sleep out & booster on
#define ST7735R_PTLON   0x12 // Partial mode on. Default after reset: off.
#define ST7735R_NORON   0x13 // Partial off (normal)
#define ST7735R_INVOFF  0x20 // Display inversion off. Default after reset: off.
#define ST7735R_INVON   0x21 // Display inversion on
#define ST7735R_GAMSET  0x26 // Gamma curve select (one data byte follows). Default after reset: GC0
  #define GAMSET_GC0    0x01 // Gamma curve 1 (2.2x if GS=1, 1.0x otherwise)
  #define GAMSET_GC1    0x02 // Gamma curve 2 (1.8x if GS=1, 2.5x otherwise)
  #define GAMSET_GC2    0x04 // Gamma curve 3 (2.5x if GS=1, 2.2x otherwise)
  #define GAMSET_GC3    0x08 // Gamma curve 4 (1.0x if GS=1, 1.8x otherwise)
#define ST7735R_DISPOFF 0x28 // Display off. Default after reset: off.
#define ST7735R_DISPON  0x29 // Display on
#define ST7735R_CASET   0x2A // Column address set (four data bytes follow)
#define ST7735R_RASET   0x2B // Row address set (four data bytes follow)
#define ST7735R_RAMWR   0x2C // Memory write (A custom N amount of data bytes follows)
#define ST7735R_RGBSET  0x2D // LUT for 4k, 65k, 262k color (N data bytes follows)
#define ST7735R_RAMRD   0x2E // Memory read
#define ST7735R_PTLAR   0x30 // Partial start/end address set (four data bytes follow)
#define ST7735R_TEOFF   0x34 // Tearing effect line off
#define ST7735R_TEON    0x35 // Tearing effect mode set & on (one data byte follows)
#define ST7735R_MADCTL  0x36 // Memory data access control (one data byte follows)
  #define MADCTL_MH  0x04    // Horizontal refresh order. 0: left to right, 1: right to left.
  #define MADCTL_RGB 0x08    // RGB/BGR order. 0: RGB color filter panel, 1: BGR color filter panel.
  #define MADCTL_ML  0x10    // Vertical refresh order. 0: top to bottom, 1: bottom to top
  #define MADCTL_MV  0x20    // Row/Column exchange.
  #define MADCTL_MX  0x40    // Column address order.
  #define MADCTL_MY  0x80    // Row address order.
#define ST7735R_IDMOFF  0x38 // Idle mode off
#define ST7735R_IDMON   0x39 // Idle mode on
#define ST7735R_COLMOD  0x3A // Interface pixel format (one data byte follows)
  #define COLMOD_12BPP  0x03
  #define COLMOD_16BPP  0x05
  #define COLMOD_18BPP  0x06
  #define COLMOD_WRITE_16BPP 0x55 // Datasheet states: "The command 3Ah should be set at 55h when writing 16-bit/pixel data into frame memory". Not sure how this differs from COLMOD_16BPP though.
#define ST7735R_RDID1   0xDA // Read ID1 (two reads follows)
#define ST7735R_RDID2   0xDB // Read ID2 (two reads follows)
#define ST7735R_RDID2   0xDC // Read ID3 (two reads follows)

struct BMPImage
{
  uint16_t width;
  uint16_t height;
  uint32_t imageDataBlock;
  uint16_t imageDataBlockOffset;
};

extern PROGMEM const unsigned char ST7735R_Init_Sequence[];
void ST7735R_SendCommandList(const uint8_t *addr);
void ST7735R_Begin();
void ST7735R_Line(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b);
void ST7735R_DrawMonoSprite(int x, int y, const uint8_t *addr, int width, int height,
                            uint8_t lo, uint8_t hi, uint8_t bgLo, uint8_t bgHi);
void ST7735R_DrawText(int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b,
                      uint8_t bgR, uint8_t bgG, uint8_t bgB);
BMPImage LoadBMPImage(uint32_t fileBlock);
void ST7735R_DrawBMP(int x, int y, const BMPImage &image);
void ST7735R_Draw565(int x, int y, uint32_t imageDataBlock);

#ifdef ST7735R_INCLUDE_IMPLEMENTATION

// Adapted from Arduino.cc TFT library:
#define DELAY 0x80
PROGMEM const unsigned char ST7735R_Init_Sequence[] =
{
  9,
  ST7735R_SWRESET, DELAY, 10,
  ST7735R_SLPOUT, DELAY, 100,
  ST7735R_GAMSET, 1, GAMSET_GC2,
  ST7735R_COLMOD, 1 | DELAY, COLMOD_WRITE_16BPP, 20,
  ST7735R_MADCTL, 1, MADCTL_MY | MADCTL_MV | MADCTL_RGB,
  ST7735R_CASET, 4, 0, 0, 0, ST7735R_WIDTH-1,
  ST7735R_RASET, 4, 0, 0, 0, ST7735R_HEIGHT-1,
  ST7735R_NORON, DELAY, 2,
  ST7735R_DISPON, DELAY, 100
};

// Adapted from Arduino.cc TFT library:
void ST7735R_SendCommandList(const uint8_t *addr)
{
  uint8_t numCommands = pgm_read_byte(addr++);
  while(numCommands--)
  {
    SEND_COMMAND(pgm_read_byte(addr++));
    uint8_t numArgs = pgm_read_byte(addr++);
    uint8_t ms = numArgs & DELAY;
    numArgs &= ~DELAY;
    while(numArgs--) WRITE_SPI_SYNC(pgm_read_byte(addr++));
    if (ms) delay((uint16_t)pgm_read_byte(addr++)*5);
  }
}

// Initializes the ST7735R display. Call this at startup.
void ST7735R_Begin()
{
  pinMode(ST7735R_CS_PIN, OUTPUT);
  pinMode(ST7735R_RS_PIN, OUTPUT);

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV2);

  pinMode(ST7735R_RST_PIN, OUTPUT);
  digitalWrite(ST7735R_RST_PIN, HIGH);
  delay(500);
  digitalWrite(ST7735R_RST_PIN, LOW);
  delay(500);
  digitalWrite(ST7735R_RST_PIN, HIGH);
  delay(500);

  ST7735R_BEGIN_TRANSACTION();
  BEGIN_TFT();
  ST7735R_SendCommandList(ST7735R_Init_Sequence);
  END_TFT();
  ST7735R_END_TRANSACTION();
}

#endif

// Begins drawing a single rectangular array of pixels one scanline at a time.
// The end coordinaes are inclusive, i.e. the range [x0, x1] X [y0, y1] is drawn.
// Use as follows:
//   ST7735R_BeginRect(topLeftX, topLeftY, bottomRightX, bottomRightY);
//   for(int y = topLeftY; y <= bottomRightY; ++y)
//      for(int x = topLeftX; x <= bottomRightX; ++x)
//         ST7735R_PushPixel(r, g, b);
//   ST7735R_EndDraw();
#define ST7735R_BeginRect(x0, y0, x1, y1) \
  do { \
    /*BEGIN_TFT();*/ \
    SEND_COMMAND(ST7735R_CASET); /* Column address set */ \
    WRITE_SPI_SYNC(0); /* XSTART[15:8] */ \
    WRITE_SPI_SYNC(x0); /* XSTART[7:0] */ \
    WRITE_SPI_SYNC(0); /* XEND[15:8] */ \
    WRITE_SPI_SYNC(x1); /* XEND[7:0] */ \
    SEND_COMMAND(ST7735R_RASET); \
    WRITE_SPI_SYNC(0); /* YSTART[15:8] */ \
    WRITE_SPI_SYNC(y0); /* YSTART[7:0] */ \
    /* hack: The YEND coordinate is always parked to the bottom of the screen. */ \
    /*WRITE_SPI_SYNC(0);*/ /* YEND[15:8] */ \
    /*WRITE_SPI_SYNC(y1);*/ /* YEND[7:0] */ \
    SEND_COMMAND(ST7735R_RAMWR); /* RAM Write */ \
  } while(0)

// Begins drawing a batch of individual pixels.
// Use as follows:
//   ST7735R_BeginPixels();
//   for(int i = 0; i < severalTimes; ++i)
//      ST7735R_Pixel(x, y, r, g, b); 
//   ST7735R_EndDraw();
#define ST7735R_BeginPixels() ST7735R_BeginRect(0, 0, 159, 127)

#define ST7735R_EndDraw() WAIT_SPI

// Computes the high 8 bits of a 24-bit RGB triplet converted to a R5G6B5 quantity.
#define HI8_RGB24(r,g,b) (((uint8_t)(r) & 0xF8) | ((uint8_t)(g) >> 5))
// Computes the low 8 bits of a 24-bit RGB triplet converted to a R5G6B5 quantity.
#define LO8_RGB24(r,g,b) ((((uint8_t)(g) << 3) & 0xE0) | ((uint8_t)(b) >> 3))

// Plots a single pixel on the screen. Call this function only in between
// a ST7735R_BeginPixels() and ST7735R_EndDraw() block.
// x: Horizontal coordinate in the range [0, 159].
// y: Vertical coordinate [0, 127].
// r, g, b: Color components in the range [0, 255] (24-bit color).
#define ST7735R_Pixel(x, y, r, g, b) \
  do { \
    WAIT_SPI; \
    SEND_COMMAND(ST7735R_CASET); \
    WRITE_SPI_SYNC(0); /*XSTART[15:8]*/ \
    WRITE_SPI_SYNC((uint8_t)(x)); /*XSTART[7:0]*/ \
    /* hack: the data sheet states that CASET instruction should be     */ \
    /* followed by four data bytes, two for XSTART and two for XEND,    */ \
    /* practice shows that we can omit sending the two XEND bytes here, */ \
    /* which speeds up performance. We "park" the XEND and YEND to      */ \
    /* bottom right corner of the screen, and only ever adjust top-left */ \
    /* coordinates. It is uncertain if this is allowed by the spec, but */ \
    /* works ok on my tests. If you run into problems, uncomment the    */ \
    /* following two lines.                                             */ \
    /* WRITE_SPI_SYNC(0);*/ /*XEND[15:8]*/ \
    /* WRITE_SPI_SYNC(ST7735R_WIDTH-1);*/ /*XEND[7:0]*/ \
    SEND_COMMAND(ST7735R_RASET); \
    WRITE_SPI_SYNC(0); \
    WRITE_SPI_NOWAIT((uint8_t)(y)); \
    /* hack: same as above. If you run into problems, uncomment below: */ \
    /* NOP; WAIT_SPI; WRITE_SPI_SYNC(0);*/ /*YEND[15:8]*/ \
    /* WRITE_SPI_NOWAIT(ST7735R_HEIGHT-1);*/ /*YEND[7:0]*/ \
    uint8_t hi = HI8_RGB24(r,g,b); \
    WAIT_SPI; \
    SEND_COMMAND(ST7735R_RAMWR); \
    WRITE_SPI_NOWAIT(hi); \
    uint8_t lo = LO8_RGB24(r,g,b); \
    WAIT_SPI; \
    WRITE_SPI_NOWAIT(lo); \
  } while(0)

// Specifies the next pixel in a rectangular grid of pixels. Call this function
// only in between a ST7735R_BeginRect() and ST7735R_EndDraw() block.
// r,g,b: Color of the pixel, each value in the range [0, 255], i.e. 24-bit colors.
#define ST7735R_PushPixel(r,g,b) \
  do { \
    uint8_t hi = HI8_RGB24(r,g,b); \
    WAIT_SPI; \
    WRITE_SPI_NOWAIT(hi); \
    uint8_t lo = LO8_RGB24(r,g,b); \
    WAIT_SPI; \
    WRITE_SPI_NOWAIT(lo); \
  } while(0)

// Specifies the next pixel in a rectangular grid of pixels, in two 8-bit halves of the
// 16-bit RGB565 color. Call this function only in between a ST7735R_BeginRect()
// and ST7735R_EndDraw() block.
// This function can be used as a replacement for ST7735R_PushPixel() wherever
// that function is accepted. This function is a tiny bit faster in the case when
// doing single color fills, in which case one can precompute the color up front.
// i.e. the two code snippets are equivalent:
//
//    ST7735R_PushPixel(r,g,b);
//
//  vs
//
//    uint8_t lo = LO8_RGB24(r,g,b);
//    uint8_t hi = HI8_RGB24(r,g,b);
//    ST7735R_PushPixel_U16(lo, hi);
#define ST7735R_PushPixel_U16(lo, hi) \
  do { \
    NOP; \
    WAIT_SPI; \
    WRITE_SPI_SYNC(hi); \
    WRITE_SPI_NOWAIT(lo); \
  } while(0)

#define ST7735R_HLine_U16(x0, x1, y, lo, hi) \
  do { \
    ST7735R_BeginRect(x0, y, x1, y); \
    for(uint8_t i = x0; i <= x1; ++i) \
      ST7735R_PushPixel_U16(lo, hi); \
    WAIT_SPI; \
  } while(0)

#define ST7735R_HLine(x0, x1, y, r, g, b) \
  do { \
    uint8_t lo = LO8_RGB24(r,g,b); \
    uint8_t hi = HI8_RGB24(r,g,b); \
    ST7735R_HLine_U16(x0, x1, y, lo, hi); \
  } while(0)

#define ST7735R_VLine_U16(x, y0, y1, lo, hi) \
  do { \
    ST7735R_BeginRect(x, y0, x, y1); \
    for(uint8_t i = (y0); i <= (y1); ++i) \
      ST7735R_PushPixel_U16(lo, hi); \
    WAIT_SPI; \
  } while(0)

#define ST7735R_VLine(x, y0, y1, r, g, b) \
  do { \
    uint8_t lo = LO8_RGB24(r,g,b); \
    uint8_t hi = HI8_RGB24(r,g,b); \
    ST7735R_VLine_U16(x, y0, y1, lo, hi); \
  } while(0)

#define ST7735R_FillRect(x0, y0, x1, y1, r, g, b) \
  do { \
    uint8_t lo = LO8_RGB24(r,g,b); \
    uint8_t hi = HI8_RGB24(r,g,b); \
    ST7735R_BeginRect((x0), (y0), (x1), (y1)); \
    for(uint16_t i = ((y1)-(y0)+1)*((x1)-(x0)+1); i > 0; --i) { \
      /* N.B. unrolling the loop does not help here, since the loop control \
         is already done parallel to waiting for SPI operation to complete. */ \
        ST7735R_PushPixel_U16(lo, hi); \
    } \
    WAIT_SPI; \
  } while(0)

#ifdef ST7735R_INCLUDE_IMPLEMENTATION

void __attribute__((always_inline)) swap_int(int &a, int &b)
{
  int tmp = a;
  a = b;
  b = tmp;
}

void ST7735R_Line(int x0, int y0, int x1, int y1, uint8_t r, uint8_t g, uint8_t b)
{
  int dx = x1 - x0;
  int dy = y1 - y0;
  int adx = abs(dx);
  int ady = abs(dy);

  uint8_t lo = LO8_RGB24(r,g,b);
  uint8_t hi = HI8_RGB24(r,g,b);

  if (ady < adx) // "Mostly horizontal": Bresenham's algorithm in the variant where x travels faster than y.
  {
    if (x1 < x0)
    {
      swap_int(x0, x1);
      swap_int(y0, y1);
      dx = -dx;
      dy = -dy;
    }
    int ysign = (dy >= 0) ? 1 : -1;

    // General strategy here is the same as with the other graphics functions:
    // interleave SPI commands with computation as much as possible, and only
    // send the minimal number of SPI CASET/RASET commands that are needed
    // needed to traverse the cursor across the line.
    ST7735R_BeginRect(x0, y0, 255, 127);
    int diff = dy >> 1;
    while(x0 <= x1)
    {
      WAIT_SPI;
      WRITE_SPI_NOWAIT(hi);
      diff -= ady;
      ++x0;
      WAIT_SPI;
      WRITE_SPI_NOWAIT(lo);

      if (diff < 0)
      {
        y0 += ysign;
        diff += dx;
        WAIT_SPI;
        // The line is "mostly horizontal", so CASET and RASET end coordinates
        // are parked to bottom right of the screen, and only the start cursor
        // needs to be specified. This exploits the advantage that when the
        // y coordinate of two adjacent pixels doesn't change, we don't need
        // to change CASET/RASET commands either.
        SEND_COMMAND(ST7735R_CASET);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(x0);
        SEND_COMMAND(ST7735R_RASET);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(y0);
        SEND_COMMAND(ST7735R_RAMWR);
      }
    }
  }
  else // "Mostly vertical": Bresenham's algorithm in the variant where y travels faster than x.
  {
    if (y1 < y0)
    {
      swap_int(x0, x1);
      swap_int(y0, y1);
      dx = -dx;
      dy = -dy;      
    }
    int xsign = (dx >= 0) ? 1 : -1;

    ST7735R_BeginRect(x0, y0, x0, 127);

    int diff = dx >> 1;
    while(y0 <= y1)
    {
      WAIT_SPI;
      WRITE_SPI_NOWAIT(hi);
      diff -= adx;
      ++y0;
      WAIT_SPI;
      WRITE_SPI_NOWAIT(lo);

      if (diff < 0)
      {
        x0 += xsign;
        diff += dy;
        WAIT_SPI;
        // The line is "mostly vertical", so have the CASET be a single pixel
        // window, so that the cursor traverses downwards naturally. YEND is
        // parked to bottom end of the screen. Only need to send CASET/RASET
        // commands again when the x coordinate changes. Overall the
        // "mostly vertical" path is two SPI commands slower per x coord
        // delta compared to the "mostly horizontal" path above.
        SEND_COMMAND(ST7735R_CASET);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(x0);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(x0);
        SEND_COMMAND(ST7735R_RASET);
        WRITE_SPI_SYNC(0);
        WRITE_SPI_SYNC(y0);
        SEND_COMMAND(ST7735R_RAMWR);
      }
    }
  }
  WAIT_SPI;
}

void ST7735R_DrawMonoSprite(int x, int y, const uint8_t *addr, int width, int height,
                            uint8_t lo, uint8_t hi, uint8_t bgLo, uint8_t bgHi)
{
  ST7735R_BeginRect(x, y, x + width-1, ST7735R_HEIGHT-1);
  int nBytes = (width*height) >> 3; // Width * height must be divisible by 8!
  while(nBytes-- > 0)
  {
    uint8_t byte = pgm_read_byte(addr++);
    for(uint8_t bit = 1; bit; bit <<= 1)
    {
      if ((byte & bit) != 0) ST7735R_PushPixel_U16(lo, hi);
      else ST7735R_PushPixel_U16(bgLo, bgHi);
    }
  }
  ST7735R_EndDraw();
}

#ifdef ST7735R_MONACO_FONT

#include "C:/Users/clb/Downloads/monaco_font.h"

void ST7735R_DrawText(int x, int y, const char *text, uint8_t r, uint8_t g, uint8_t b,
                      uint8_t bgR, uint8_t bgG, uint8_t bgB)
{
  uint8_t ch = *text;
  uint8_t lo = LO8_RGB24(r,g,b);
  uint8_t hi = HI8_RGB24(r,g,b);
  uint8_t bgLo = LO8_RGB24(bgR,bgG,bgB);
  uint8_t bgHi = HI8_RGB24(bgR,bgG,bgB);
  while(ch)
  {
    int height_adjust = pgm_read_byte(monaco_height_adjust + ch-32);
    ST7735R_DrawMonoSprite(x, y + height_adjust, monaco_font + (ch-32) * 5, 5, 8, lo, hi, bgLo, bgHi);
    ++text;
    ch = *text;
    x += 6;
  }
}

#endif

#ifdef ST7735R_ENABLE_SDCARD

BMPImage LoadBMPImage(uint32_t fileBlock)
{
  BEGIN_SDCARD();
  BMPImage image;

  Serial.print(F("beginread of block "));
  Serial.println(fileBlock);
  SdCard_BeginRead(fileBlock);
  Serial.println(F("ok"));

  // BitmapFileHeader: (14 bytes)
  SdCard_readU16LSB(); // bfType
  uint32_t size = SdCard_readU32LSB(); // bfSize
  Serial.print(F("size: "));
  Serial.println(size);
  SdCard_readU16LSB(); // bfReserved1
  SdCard_readU16LSB(); // bfReserved2
  uint32_t byteDataPos = SdCard_readU32LSB(); // bfOffBits
  Serial.print(F("byteDataPos: "));
  Serial.println(byteDataPos);

  // BitmapInfoHeader: (20 bytes)
  uint32_t biSize = SdCard_readU32LSB(); // biSize
  Serial.print(F("biSize: "));
  Serial.println(biSize);
  image.width = (uint16_t)SdCard_readU32LSB(); // biWidth
  image.height = (uint16_t)SdCard_readU32LSB(); // biHeight
  uint16_t planes = SdCard_readU16LSB(); // biPlanes
  uint16_t bitCount = SdCard_readU16LSB(); // biBitCount
  uint32_t compression = SdCard_readU32LSB(); // biCompression

  Serial.print(F("image.width: "));
  Serial.println(image.width);
  Serial.print(F("image.height: "));
  Serial.println(image.height);

  Serial.print(F("planes: "));
  Serial.println(planes);
  Serial.print(F("bitCount: "));
  Serial.println(bitCount);
  Serial.print(F("compression: "));
  Serial.println(compression);

  image.imageDataBlock = fileBlock + (byteDataPos >> 9);
  image.imageDataBlockOffset = byteDataPos & 0x1FF;

  Serial.print(F("image.imageDataBlock: "));
  Serial.println(image.imageDataBlock);
  Serial.print(F("image.imageDataBlockOffset: "));
  Serial.println(image.imageDataBlockOffset);

  SdCard_EndRead();

  END_SDCARD();

  return image;
}

// A temporary memory area to store 16-bit color pixels from the image until these
// are flushed to the display. TODO: Either figure out if this is possible to remove
// without losing performance, or if not, adapt the functions below so that this
// header does not define this variable, but the functions ask for a scratchpad
// buffer to use so that user is in control memory allocation. (this takes up a 
// whopping 25% of Arduino Uno memory space!)
uint8_t st7735r_pixelBuffer[512];

// Streams a BMP image from the SD Card on to the screen.
// Drawing images from SD card could be optimized quite a bit if one stored the images
// in raw RGB16 format that matched the display format, which would allow directly
// streaming the bytes in a very tight loop. However, not that interested in adding
// a custom format + an image converter, so much nicer to read "standard" BMP images
// that are RGB24 and bottom up for now.
void ST7735R_DrawBMP(int x, int y, const BMPImage &image)
{
  // BMP images are commonly stored scanlines bottom up, but the default rendering
  // mode that is used for the ST7735R display is top down. Fortunately the TFT
  // screen has a special command to choose which way the pixels are filled in, so
  // temporarily flip the scanline order for the display for the duration of this
  // function call (and restore at exit).
  // TODO: Test with x != 0 and y != 0, this flipping probably doesn't work without
  // making an offset adjustment to y coordinate, but haven't tested yet except with
  // fullscreen images.
  BEGIN_TFT();
  CHOOSE_COMMAND_MODE;
  WRITE_SPI_SYNC(ST7735R_MADCTL);
  CHOOSE_DATA_MODE;
  WRITE_SPI_SYNC(MADCTL_MY | MADCTL_MV | MADCTL_RGB | MADCTL_MX);
  ST7735R_BeginRect(x, y, x + image.width - 1, y + image.height - 1);
  END_TFT();

  // Activate SD card, and start reading the block where the image data starts in.
  BEGIN_SDCARD();
  uint32_t currentBlock = image.imageDataBlock;
  SdCard_BeginRead(currentBlock);
  SdCard_SkipBytes(image.imageDataBlockOffset); // Seek to the beginning of pixel data.

  // For some reason streaming data directly from the SD card to the display by
  // reading a very small amount of data and flipping the SPI select pins to alternate
  // per-pixel does not work. Therefore in the main loop below, read a full FAT32 blocks of
  // pixel bytes at a time before flushing those on the screen.

  uint16_t numPixels = image.width * image.height; // Total number of pixels we have left to draw.
  int bufferedPixelBytes = 0;
  while(numPixels > 0)
  {
    int numBytesLeftInThisBlock = 512 - SdCard_numBytesRead;
    if (numBytesLeftInThisBlock > (uint32_t)numPixels*3) numBytesLeftInThisBlock = numPixels*3;

    uint8_t r, g, b;

    // Read the whole FAT32 block of RGB24 data in to memory, converting to RGB16 on the fly
    // to reduce memory pressure. Interleave computation with SPI writes as much as possible
    // to get free cycles while waiting for SPI to complete.
    while(numBytesLeftInThisBlock >= 3)
    {
      WRITE_SPI_NOWAIT(0xFF);
      numBytesLeftInThisBlock -= 3;
      NOP;
      WAIT_SPI;
      b = SPDR;

      WRITE_SPI_NOWAIT(0xFF);
      --numPixels;
      SdCard_numBytesRead += 3;
      NOP;
      WAIT_SPI;
      g = SPDR;

      WRITE_SPI_NOWAIT(0xFF);
      st7735r_pixelBuffer[bufferedPixelBytes+1] = LO8_RGB24(0,g,b);
      WAIT_SPI;
      r = SPDR;

      st7735r_pixelBuffer[bufferedPixelBytes] = HI8_RGB24(r,g,0);
      bufferedPixelBytes += 2;
    }

    // The full block has now been read, but since there's 24 bits to each pixel in the BMP
    // image, have to do this somewhat inconvenient management for pixels that straddle a
    // block boundary.
    if (numBytesLeftInThisBlock >= 1)
    {
      // Read the last partial bytes of a RGB triplet that will continue on the beginning
      // of the next FAT32 block.
      b = SdCard_readU8();
      if (numBytesLeftInThisBlock == 2)
        g = SdCard_readU8();
    }
    SdCard_EndRead();

    // Switch SPI operation from SD card to the TFT display and flush the pixels that we
    // have read so far on to the screen.
    END_SDCARD();
    BEGIN_TFT();
    for(int i = 0; i < bufferedPixelBytes; ++i)
      WRITE_SPI_SYNC(st7735r_pixelBuffer[i]);
    bufferedPixelBytes = 0;
    END_TFT();
    BEGIN_SDCARD();

    // If we still haven't finished but have more pixels to draw, prepare reading the next
    // FAT32 block from the SD card.
    if (numPixels > 0)
    {
      ++currentBlock;
      SdCard_BeginRead(currentBlock);

      // If there was a partial pixel straddling the block boundary, complete that in first.
      if (numBytesLeftInThisBlock >= 1)
      {
        if (numBytesLeftInThisBlock == 1)
          g = SdCard_readU8();
        r = SdCard_readU8();

        st7735r_pixelBuffer[bufferedPixelBytes++] = HI8_RGB24(r,g,b);
        st7735r_pixelBuffer[bufferedPixelBytes++] = LO8_RGB24(r,g,b);
        --numPixels;
      }
      numBytesLeftInThisBlock = 512 - SdCard_numBytesRead;
    }
  }
  // Finish partial block read, and flush any still unflushed pixels to the screen.
  SdCard_EndRead();
  END_SDCARD();

  BEGIN_TFT();
  for(int i = 0; i < bufferedPixelBytes; ++i)
    WRITE_SPI_SYNC(st7735r_pixelBuffer[i]);
  CHOOSE_COMMAND_MODE;
  WRITE_SPI_SYNC(ST7735R_MADCTL);
  CHOOSE_DATA_MODE;
  WRITE_SPI_SYNC(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
  END_TFT();
}

// Streams a "565 image" from the SD Card on to the screen.
// A 565 image is one that has been converted already to a suitable format for streaming.
// The structure consists of 16-bit words as follows:
// word 0 (Little Endian): width
// word 1 (Little Endian): height
// word 3: dummy padding, zero, unused
// word 4: dummy padding, zero, unused
// words [5, 5+width*height-1] (big endian): pixel data, in x major order, top to bottom.
// Note: To gain extra few percent of performance, the loops have been manually unrolled
// to do four pixels at a time, which means that width*height must be divisible by four!
// (really silly requirement, but this library is about squeezing every bit of performance,
// so doesn't really matter. To remove this restriction, either add tail handling, after
// the unrolled loops (e.g. Duff's device), or remove the unrolling)
void ST7735R_Draw565(int x, int y, uint32_t imageDataBlock)
{
  END_TFT();
  // Activate SD card, and start reading the block where the image data starts in.
  BEGIN_SDCARD();
  uint32_t currentBlock = imageDataBlock;
  SdCard_BeginRead(currentBlock);

  uint16_t width = SdCard_readU16LSB();
  uint16_t height = SdCard_readU16LSB();
  bool firstBlock = true;
  // Two bytes of dummy padding to make the unrolling align up.
  SdCard_readU16LSB();
  SdCard_readU16LSB();
  uint16_t numPixels = width * height; // Total number of pixels we have left to draw.
  int numPixelsLeftInThisBlock = (512 - 4*sizeof(uint16_t))>>1;
  if (numPixelsLeftInThisBlock > numPixels)
    numPixelsLeftInThisBlock = numPixels;

  while(numPixels > 0)
  {
    numPixels -= numPixelsLeftInThisBlock;
    // Unroll to read 4 pixels at a time (8 bytes). This also makes numIters small
    // enough to fit into a uint8_t for optimal one cycle loop counter.
    const uint8_t numIters = (uint8_t)(numPixelsLeftInThisBlock >> 2);
    uint8_t *buf = st7735r_pixelBuffer;
    for(uint8_t i = 0; i < numIters; ++i)
    {
      WRITE_SPI_SYNC(0xFF); *buf++ = SPDR;
      WRITE_SPI_SYNC(0xFF); *buf++ = SPDR;
      WRITE_SPI_SYNC(0xFF); *buf++ = SPDR;
      WRITE_SPI_SYNC(0xFF); *buf++ = SPDR;
      WRITE_SPI_SYNC(0xFF); *buf++ = SPDR;
      WRITE_SPI_SYNC(0xFF); *buf++ = SPDR;
      WRITE_SPI_SYNC(0xFF); *buf++ = SPDR;
      WRITE_SPI_SYNC(0xFF); *buf++ = SPDR;
    }
    SdCard_numBytesRead += (uint16_t)numIters<<3;
    SdCard_EndRead();

    // Switch SPI operation from SD card to the TFT display and flush the pixels that we
    // have read so far on to the screen.
    END_SDCARD();
    BEGIN_TFT();
    if (firstBlock)
    {
      ST7735R_BeginRect(x, y, x + width - 1, y + height - 1);
      firstBlock = false;
    }
    buf = st7735r_pixelBuffer;
    for(uint8_t i = 0; i < numIters; ++i)
    {
      WRITE_SPI_SYNC(*buf++);
      WRITE_SPI_SYNC(*buf++);
      WRITE_SPI_SYNC(*buf++);
      WRITE_SPI_SYNC(*buf++);
      WRITE_SPI_SYNC(*buf++);
      WRITE_SPI_SYNC(*buf++);
      WRITE_SPI_SYNC(*buf++);
      WRITE_SPI_SYNC(*buf++);
    }
    END_TFT();
    BEGIN_SDCARD();

    // If we still haven't finished but have more pixels to draw, prepare reading the next
    // FAT32 block from the SD card.
    if (numPixels > 0)
    {
      SdCard_BeginRead(++currentBlock);
      numPixelsLeftInThisBlock = min(numPixels, 256);
    }
  }
  END_SDCARD();
}
#endif // ~ST7735R_ENABLE_SDCARD

#endif // ~ST7735R_INCLUDE_IMPLEMENTATION
