void setup_test()
{
}

void loop_test()
{
  za = 1;
  zb = 10;
  zc = 100;
  zx = 50;
  for(int i = 0; i < 100; ++i)
  {
    int x0, x1;
    do { x0 = rng(); } while(x0 >= ST7735R_WIDTH);
    int y0 = rng() & (ST7735R_HEIGHT-1);
    do { x1 = rng(); } while(x1 >= ST7735R_WIDTH);
    int y1 = rng() & (ST7735R_HEIGHT-1);
    
    uint8_t r = rng();
    uint8_t g = rng();
    uint8_t b = rng();

#ifdef USE_ARDUINO_TFT
    tft.stroke(r, g, b);
    tft.line(x0, y0, x1, y1);
#else
    ST7735R_Line(x0, y0, x1, y1, r, g, b);
#endif
  }
}
