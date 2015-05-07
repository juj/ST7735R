//rotate/flip a quadrant appropriately
void rot(int n, int *x, int *y, int rx, int ry) {
    if (ry == 0) {
        if (rx == 1) {
            *x = n-1 - *x;
            *y = n-1 - *y;
        }
 
        //Swap x and y
        int t  = *x;
        *x = *y;
        *y = t;
    }
}

//convert d to (x,y)
void d2xy(int n, int d, int *x, int *y) {
    int rx, ry, s, t=d;
    *x = *y = 0;
    for (s=1; s<n; s*=2) {
        rx = 1 & (t/2);
        ry = 1 & (t ^ rx);
        rot(s, x, y, rx, ry);
        *x += s * rx;
        *y += s * ry;
        t /= 4;
    }
}

void setup_test()
{
}

int i = 0;
int prev_x, prev_y;

uint8_t r = 255, g = 0, b = 0;
void loop_test()
{
#if !defined(USE_ARDUINO_TFT)
  ST7735R_BeginPixels();
#endif
#ifdef USE_ARDUINO_TFT
  tft.stroke(r, g, b);
#endif
  
  int x, y;
  for(int j = 0; j < 64*64; ++j)
  {
    d2xy(64, i, &x, &y);
    x = 2*x + 16;
    y *= 2;
    
    if (i != 0)
    {
#ifdef USE_ARDUINO_TFT
      tft.line(prev_x, prev_y, x, y);
//      tft.point((prev_x + x) >> 1, (prev_y + y) >> 1);
//      tft.point(x, y);
#else
//      ST7735R_Line(prev_x, prev_y, x, y, r, g, b);
      ST7735R_Pixel((prev_x + x) >> 1, (prev_y + y) >> 1, r, g, b);
      ST7735R_Pixel(x, y, r, g, b);
#endif
    }
    prev_x = x;
    prev_y = y;
    ++i;
    if (i >= 64*64)
      i = 0;
  }
  uint8_t a = r;
  r = b;
  b = g;
  g = a;
  
#if !defined(USE_ARDUINO_TFT)
  ST7735R_EndDraw();
#endif

}
