void setup_test()
{
}

void loop_test()
{
  uint8_t r = rng();
  uint8_t g = rng();
  uint8_t b = rng();
#ifdef USE_ARDUINO_TFT
  tft.background(r, g, b);
#else
  ST7735R_FillRect(0, 0, ST7735R_WIDTH-1, ST7735R_HEIGHT-1, r, g, b);
#endif
  
}
