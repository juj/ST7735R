#if defined(USE_ARDUINO_TFT)
PImage bmpImage[3];
#else

#if !defined(ST7735R_ENABLE_SDCARD)
#error Please #define ST7735R_ENABLE_SDCARD at the top of testrunner.h!
#endif

BMPImage bmpImage[3];
#endif

int idx;

void setup_test()
{
#if defined(USE_ARDUINO_TFT)
  bmpImage[0] = tft.loadImage("outrun.bmp");
  bmpImage[1] = tft.loadImage("outrun2.bmp");
  bmpImage[2] = tft.loadImage("outrun3.bmp");
#else
  BEGIN_SDCARD();
  END_TFT();
  bmpImage[0] = LoadBMPImage(SDCard_GetFileStartingBlock("outrun.bmp"));
  bmpImage[1] = LoadBMPImage(SDCard_GetFileStartingBlock("outrun2.bmp"));
  bmpImage[2] = LoadBMPImage(SDCard_GetFileStartingBlock("outrun3.bmp"));
  END_SDCARD();
#endif
  idx = 0;
}

void loop_test()
{
#if defined(USE_ARDUINO_TFT)
  tft.image(bmpImage[idx], 0, 0);
#else
  ST7735R_DrawBMP(0, 0, bmpImage[idx]);
#endif
  idx = (idx+1)%3;
}

