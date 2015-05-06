#pragma once

//#define USE_ARDUINO_TFT

// Enable this #define when building any of the tests 6-7.
//#define ST7735R_ENABLE_SDCARD

// Enable this feature to draw a performance meter, or when building test 3.
#define ST7735R_MONACO_FONT

#ifdef ST7735R_ENABLE_SDCARD
#include <SD.h>
#endif

#ifdef USE_ARDUINO_TFT
#include <TFT.h>
#include <SPI.h>
#endif

#include "testrunner.h"

//#include "Test1_Fillscreen.h"
//#include "Test2_Blocks.h"
//#include "Test3_WanderingText.h"
#include "Test4_Starfield.h"
//#include "Test5_Lines.h"

// The following tests require the ST7735R_ENABLE_SDCARD define
// to be uncommented at the top of the file.

//#include "Test6_DrawBMP.h"
//#include "Test7_Draw565.h"

