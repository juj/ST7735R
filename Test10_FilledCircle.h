void setup_test()
{
}

void loop_test()
{
#ifdef USE_ARDUINO_TFT
  tft.noStroke();
#endif

  for(int r = 63; r > 0; --r)
  {
#ifdef USE_ARDUINO_TFT
    tft.fill(rng(), 0, 0);
    tft.circle(80, 64, r); 
#else
    ST7735R_FilledCircle(80, 64, r, rng(), 0, 0);
#endif
  }
}

