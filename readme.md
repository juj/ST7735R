![TFT display](/pics/logo.jpg "Arduino ST7735R TFT")

This repository contains a fast low level Arduino Uno compatible graphics library for the 160x128 pixel 16-bit color TFT LCD display that uses the ST7735R chip. It was written as a programming exercise after realizing that the built-in "Adafruit" TFT library that Arduino ships with is very slow. I wanted to display some slideshow images on the TFT using an Arduino, but unfortunately the built-in libraries take about 2.9 seconds(!) to draw a single 160x128 image on the screen from the SD card. With this library, it is possible to draw an image in about 188 milliseconds, which is a considerable 15.2x performance increase! The aim of this code is to squeeze every individual clock cycle out of the drawing routines to see how fast it is possible to drive the ST7735R display. If you can improve the code here, please let me know.

#### Rendering features

This library offers (almost) drop-in replacements for most built-in TFT library methods, and comes with the following functions:
  - Draw batches of individual pixels with a sequence of `ST7735R_BeginPixels()`, `ST7735R_Pixel()` and `ST7735R_EndDraw()` functions.
  - Fill solid rectangles with the `ST7735R_FillRect()` function.
  - Stream rectangles of custom content with a sequence of `ST7735R_BeginRect()`, `ST7735R_PushPixel()` and `ST7735R_EndDraw()` functions.
  - Draw horizontal lines with the function `ST7735R_HLine()` and vertical lines with the function `ST7735R_VLine()`.
  - Draw arbitrary sloped lines with the function `ST7735R_Line()`.
  - Draw hollow circles with the function `ST7735R_Circle()`.
  - Draw filled circles with the function `ST7735R_FilledCircle()`.
  - Draw 1-bit monochrome sprites from PROGMEM with the function `ST7735R_DrawMonoSprite()`.
  - Draw text stored in PROGMEM with the function `ST7735R_DrawText()`. One font file `"monaco_font.h"` is provided, create more with the `ttffont_to_cppheader.py` tool.
  - Draw 24-bit bottom-up .bmp images with the functions `LoadBMPImage()`, `SDCard_GetFileStartingBlock()` and `ST7735R_DrawBMP()`. (requires hacking of the built-in SD card library to enable raw FAT32 block streaming).
  - Draw pre-prepared raw 16-bit images (files formatted with suffix .565) with the functions `SDCard_GetFileStartingBlock()` and `ST7735R_Draw565()`. (requires hacking of the built-in SD card library to enable raw FAT32 block streaming). Use the python tool `image_to_rgb565.py` to convert images to this format.

The library has only been used on an Arduino Uno with 8MHz hardware SPI mode, and contains several assumptions that might not hold when transferring the configuration to other setups. Don't use blindly without understanding the internals of the code!

#### References

 - The board: http://www.arduino.cc/en/Guide/TFT (bought from http://www.verkkokauppa.com/fi/product/30873/dsgrv/Arduino-1-77-TFT-LCD-Screen-naytto-Arduino-kehitysalustoille)
 - Wiring instructions at http://www.arduino.cc/en/Guide/TFTtoBoards
 - Data sheet at http://www.adafruit.com/datasheets/ST7735R_V0.2.pdf
 - Atmel AVR 8 bit instruction set documentation: http://www.atmel.com/images/doc0856.pdf
 - Requires manual pin to port+bit mapping configuration. This is useful: http://pighixxx.com/unov3pdf.pdf

#### License and Usage

Unless otherwise stated in individual files, all code is released to public domain. Do whatever you wish with it. This repository is a result of some **recreational hacking** activity, rather than a mission to build a stable and maintained library, so expect the maturity to be as such.

#### Sample code

The files `Testx_yyyyy.h` implement different test rendering patterns for the library. To run one of these tests, open the file `ST7735R.ino` in the Arduino IDE, and uncomment the appropriate `#include "Testx_yyyyy.h"` lines to build that test. Configure the file accordingly. In order for the default configuration to work, you must run on Arduino Uno and have the pins connected according to the order recommended here: http://www.arduino.cc/en/Guide/TFTtoBoards . Otherwise edit the `#define`s at the top of the files `ST7735R_TFT.h` and `SDCardBlockRead.h`.

##### Test1_Fillscreen

![Test1_Fillscreen](/pics/Test1_Fillscreen.jpg "Test1_Fillscreen")

This test measures rectangle filling performance, and compares the Arduino `TFT::background()` method to its replacement function `ST7735R_FillRect()` in this library.

##### Test2_Blocks

![Test2_Blocks](/pics/Test2_Blocks.jpg "Test2_Blocks")

This test measures the performance of drawing multiple small rectangles, and compares the Arduino `TFT::point()` method to `ST7735R_FillRect()` in this library.

##### Test3_WanderingText

![Test3_WanderingText](/pics/Test3_WanderingText.jpg "Test3_WanderingText")

This test is a comparison of the Arduino `TFT::text()` function against the `ST7735R_DrawText()` function in this library.

##### Test4_Starfield

![Test4_Starfield](/pics/Test4_Starfield.jpg "Test4_Starfield")

This test measures the performance of drawing individual pixels with `TFT::point()` against the `ST7735R_Pixel()` function in this library. In this library, individual pixel rendering is batched, and must be enclosed between a block of calls to the functions `ST7735R_BeginPixels()` and `ST7735R_EndDraw()`.

##### Test5_Lines

![Test5_Lines](/pics/Test5_Lines.jpg "Test5_Lines")

This test draws arbitrarily slanted lines using either the Arduino function `TFT::line()`, or the function `ST7735R_Line()` in this library.

##### Test6_DrawBMP

![Test6_DrawBMP](/pics/Test6_DrawBMP.jpg "Test6_DrawBMP")

In this sample, a slideshow of full screen .bmp images are rendered with the functions `TFT::loadImage` + `TFT::image`, or with the functions `LoadBMPImage(SDCard_GetFileStartingBlock(filename))` and `ST7735R_DrawBMP()`.

##### Test7_Draw565

![Test7_Draw565](/pics/Test7_Draw565.jpg "Test7_Draw565")

This test demonstrates how a custom prepared file format can be used to accelerate imare rendering even further. The functions `SDCard_GetFileStartingBlock()` and `ST7735R_Draw565()` are used here.

##### Test8_HilbertCurve

![Test8_HilbertCurve](/pics/Test8_HilbertCurve.jpg "Test8_HilbertCurve")

This sample code draws small polyline segments and compares the tradeoff of rendering individual points versus lines.

##### Test9_HypnoCircle

![Test9_HypnoCircle](/pics/Test9_HypnoCircle.jpg "Test9_HypnoCircle")

This test demonstrates the rendering of hollow circles, either using the built-in `TFT::circle()` function (with the `noFill` option), or the function `ST7735R_Circle()` found in this library.

##### Test10_FilledCircle

![Test10_FilledCircle](/pics/Test10_FilledCircle.jpg "Test10_FilledCircle")

This test demonstrates the rendering of solid filled circles, either using the built-in `TFT::circle()` function (with the `noStroke` option), or the function `ST7735R_FilledCircle()` from in this library.

#### Performance Comparison

Toggle the `#define USE_ARDUINO_TFT` in `ST7735R.ino` to compare performance between the built-in Arduino "Adafruit" TFT library and this library. Each test measures the total time to render per frame in microseconds. That gives several data points to compare the performance of this library against the built-in library.

| Test                | Adafruit            | This library       | Relative    |
| ------------------- | ------------------- | ------------------ | ----------- |
| Test1_FillScreen    |  105752 &micro;secs |  54072 &micro;secs |  **-48.9%** |
| Test2_Blocks        |  524184 &micro;secs |  20480 &micro;secs |  **-96.0%** |
| Test3_WanderingText |  218328 &micro;secs |  28556 &micro;secs |  **-86.9%** |
| Test4_Starfield     |   68302 &micro;secs |  25670 &micro;secs |  **-62.4%** |
| Test5_Lines         |  592560 &micro;secs |  61460 &micro;secs |  **-89.6%** |
| Test6_DrawBMP       | 2858852 &micro;secs | 318028 &micro;secs |  **-88.9%** |
| Test7_Draw565       |                 N/A | 187820 &micro;secs |           - |
| Test8_HilbertCurve  |  576948 &micro;secs | 324492 &micro;secs |  **-43.8%** |
| Test9_HypnoCircle   |  915260 &micro;secs | 111356 &micro;secs |  **-87.8%** |
| Test10_FilledCircle | 2082480 &micro;secs | 933720 &micro;secs |  **-55.2%** |

Overall the data shows that one can expect a performance increase from 2x to 10x by bypassing the built-in TFT library.