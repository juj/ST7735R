#ifdef USE_ARDUINO_TFT
#error This test cannot be run in USE_ARDUINO_TFT mode!
#endif

#if !defined(ST7735R_ENABLE_SDCARD)
#error Please #define ST7735R_ENABLE_SDCARD at the top of testrunner.h!
#endif

int idx = 0;

#define NFRAMES 15

uint32_t imageBlocks[NFRAMES];

void setup_test()
{
  END_TFT();
  char filename[] = "out00230.565";
  for(int j = 0; j < NFRAMES; ++j)
  {
    for(int skip = 0; skip < 20; ++skip)
    {    
      for(int i = 7; i >= 3; --i)
      {
        if (filename[i] < '9')
        {
          filename[i]++;
          break;
        }
        else
          filename[i] = '0';
      }
    }
    imageBlocks[j] = SDCard_GetFileStartingBlock(filename);
    if (imageBlocks[j] != -1)
    {
      Serial.print("File ");
      Serial.print(filename);
      Serial.print(" starts at FAT32 block ");
      Serial.println(imageBlocks[j]);
    }
  }
}

void loop_test()
{
  ST7735R_Draw565(0, 0, imageBlocks[idx]);
  if (++idx >= NFRAMES)
    idx = 0;
}

