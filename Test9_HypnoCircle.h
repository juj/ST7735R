void setup_test()
{
}

void loop_test()
{
#ifdef USE_ARDUINO_TFT
  tft.noFill();
#endif
  for(int r = 0; r < 63; ++r)
  {
#ifdef USE_ARDUINO_TFT
    tft.stroke(0, rng(), 0);
    tft.circle(80, 64, r); 
#else
    ST7735R_Circle(80, 64, r, 0, rng(), 0);
#endif
  }
}

