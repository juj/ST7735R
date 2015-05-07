#define NSTARS 300
uint8_t sx[NSTARS] = {};
uint8_t sy[NSTARS] = {};
uint8_t sz[NSTARS] = {};

void setup_test()
{
}

void loop_test()
{
#if !defined(USE_ARDUINO_TFT)
  ST7735R_BeginPixels();
#endif
  uint8_t spawnDepthVariation = 255;
  for(int i = 0; i < NSTARS; ++i)
  {
    if (sz[i] <= 1)
    {
      sx[i] = 80 - 64 + (rng() & 127);
      sy[i] = rng() & 127;
      sz[i] = spawnDepthVariation--;
    }
    else
    {
      int old_screen_x = ((int)sx[i] - 80) * 256 / sz[i] + 80;
      int old_screen_y = ((int)sy[i] - 64) * 256 / sz[i] + 64;
#ifdef USE_ARDUINO_TFT
      tft.stroke(0,0,0);
      tft.point(old_screen_x, old_screen_y);
#else
      ST7735R_Pixel(old_screen_x, old_screen_y, 0, 0, 0);
#endif

      sz[i] -= 2;
      if (sz[i] > 1)
      {
        int screen_x = ((int)sx[i] - 80) * 256 / sz[i] + 80;
        int screen_y = ((int)sy[i] - 64) * 256 / sz[i] + 64;
  
        if (screen_x >= 0 && screen_y >= 0 && screen_x < ST7735R_WIDTH && screen_y < ST7735R_HEIGHT)
        {
          uint8_t r, g, b;
          r = g = b = 255 - sz[i];
          b = (b < 128) ? (b*2) : 255;
#ifdef USE_ARDUINO_TFT
          tft.stroke(r,g,b);
          tft.point(screen_x, screen_y);
#else
          ST7735R_Pixel(screen_x, screen_y, r, g, b);
#endif
        }
        else
          sz[i] = 0; // Out of screen, die.
      }
    }
  }
#if !defined(USE_ARDUINO_TFT)
  ST7735R_EndDraw();
#endif
}

