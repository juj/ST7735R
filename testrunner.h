#ifdef USE_ARDUINO_TFT

#define cs   10
#define dc   9
#define rst  8
TFT tft = TFT(cs, dc, rst);

#endif

#include "ST7735R_TFT.h"

#ifdef ST7735R_ENABLE_SDCARD
#include <SD.h>
#endif

// The "XABC" random number generator from http://eternityforest.com/Projects/rng.php:
uint8_t za, zb, zc, zx;
uint8_t __attribute__((always_inline)) rng()
{
  zx++;
  za = (za^zc^zx);
  zb = (zb+za);
  zc = (zc+(zb>>1)^za);
  return zc;
}

void setup_test();
void loop_test();

void setup() {
  Serial.begin(9600);

#ifdef ST7735R_ENABLE_SDCARD
  END_TFT();
  if (!SD.begin(4)) {
    Serial.println(F("SD card initialization failed!"));
    return;
  }
  END_SDCARD();
#endif
  
  za = random(256);
  zb = random(256);
  zc = random(256);
  zx = random(256);

#ifdef USE_ARDUINO_TFT
  tft.begin();
  tft.background(0, 0, 0);
  
  // To make the comparison fairer, force SPI clock mode to use 8 MHz/divider 2
  // on the Arduino TFT as well, like it does on the ST7735R. The TFT library
  // will otherwise use 4 MHz/divider 4 by default, which is twice as slow.
  SPI.setClockDivider(SPI_CLOCK_DIV2);
#else
  ST7735R_Begin();
  BEGIN_TFT();
  ST7735R_FillRect(0, 0, ST7735R_WIDTH-1, ST7735R_HEIGHT-1, 0, 0, 0);
  END_SDCARD();
#endif

  setup_test();
}

void loop()
{
#if !defined(USE_ARDUINO_TFT)
  END_SDCARD();
  ST7735R_BEGIN_TRANSACTION();
  BEGIN_TFT();
#endif

  unsigned long t0 = micros();
  loop_test();
  unsigned long t1 = micros();
  static char timeMicros[8] = {};
#ifdef USE_ARDUINO_TFT
  // Erase old text.
  tft.stroke(0,0,0);
  tft.text(timeMicros, 0, 30);
#else
  BEGIN_TFT();
  ST7735R_DrawText(0, 30, (const char *)timeMicros, 0, 0, 0, 0, 0, 0);
#endif  
  sprintf(timeMicros, "%lu", t1 - t0);

#ifdef USE_ARDUINO_TFT
  tft.stroke(255, 255, 255);
  tft.text(timeMicros, 0, 30);
#else
#ifdef ST7735R_MONACO_FONT
  BEGIN_TFT();
  ST7735R_DrawText(0, 30, (const char *)timeMicros, 255, 255, 255, 0, 0, 0);
#endif
  ST7735R_END_TRANSACTION();
  END_TFT();
#endif
}

