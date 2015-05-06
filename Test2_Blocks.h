int x, y;

const int blockSize = 8;

void setup_test()
{
  x = y = 0;
}

void loop_test()
{
  for(int i = 0; i < 100; ++i)
  {
    uint8_t r = rng();
    uint8_t g = rng();
    uint8_t b = rng();
#ifdef USE_ARDUINO_TFT
    tft.stroke(r, g, b);
    for(int Y = y; Y < y+blockSize; ++Y)
      for(int X = x; X < x+blockSize; ++X)
        tft.point(X, Y);
#else
    ST7735R_FillRect(x, y, x+blockSize-1, y+blockSize-1, r, g, b);
#endif
    switch(rng() & 3)
    {
    	case 0: x = (x + blockSize) % ST7735R_WIDTH; break;
    	case 1: y = (y + blockSize) % ST7735R_HEIGHT; break;
    	case 2: x = (x + ST7735R_WIDTH - blockSize) % ST7735R_WIDTH; break;
    	case 3: y = (y + ST7735R_HEIGHT - blockSize) % ST7735R_HEIGHT; break;
    }
  }
}
