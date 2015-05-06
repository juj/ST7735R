#ifndef ST7735R_MONACO_FONT
#error Please #define ST7735R_MONACO_FONT at the top of ST7735R.ino file!
#endif

int tx[96];
int ty[96];
uint8_t tr[96];
uint8_t tg[96];
uint8_t tb[96];

void setup_test()
{
  for(int i = 0; i < 96; ++i)
  {
    tx[i] = ST7735R_WIDTH/2;
    ty[i] = ST7735R_HEIGHT/2;
    tr[i] = random(256);
    tg[i] = random(256);
    tb[i] = random(256);
    if (tr[i] < 128 && tg[i] < 128 && tb[i] < 128)
    {
      tr[i] = 255 - tr[i];
      tg[i] = 255 - tg[i];
      tb[i] = 255 - tb[i];
    }
  }
#ifdef USE_ARDUINO_TFT
  tft.setTextSize(1);
#endif
}

void loop_test()
{
  // set the font color
  // print the sensor value
  
  char ch[2] = { 'a', 0 };
  for(int i = 0; i < 96; ++i)
  {
    ch[0] = ' ' + i;
    uint8_t lcg = rng();
#ifdef USE_ARDUINO_TFT
    tft.stroke(0, 0, 0);
    tft.text(ch, tx[i], ty[i]);
#else
    ST7735R_DrawText(tx[i], ty[i], ch, 0, 0, 0, 0, 0, 0);
#endif
    if ((lcg & 8))
    {
      tx[i] += 1;
      if (tx[i] > ST7735R_WIDTH-8) tx[i] = ST7735R_WIDTH-8;
    }
    else
    {
      tx[i] -= 1;
      if (tx[i] < 0) tx[i] = 0;
    }
    lcg = rng();
    if ((lcg & 4))
    {
      ty[i] += 1;
      if (ty[i] > ST7735R_HEIGHT-8) ty[i] = ST7735R_HEIGHT-8;
    }
    else
    {
      ty[i] -= 1;
      if (ty[i] < 1) ty[i] = 1;
    }

#ifdef USE_ARDUINO_TFT
    tft.stroke(tr[i], tg[i], tb[i]);
    tft.text(ch, tx[i], ty[i]);
#else
    ST7735R_DrawText(tx[i], ty[i], ch, tr[i], tg[i], tb[i], 0, 0, 0);
#endif
  }

/*
  ST7735R_DrawText(0, 2,  " !\"$%&'()*+,-./0123456789", r, g, b, 0, 0, 0);
  ST7735R_DrawText(0, 10,  ":;<=>?@ABCDEFGHIJKLMNOPQRS", r, g, b, 0, 0, 0);
  ST7735R_DrawText(0, 18, "TUVWXYZ[\\]^_`abcdefghijklm", r, g, b, 0, 0, 0);
  ST7735R_DrawText(0, 26, "nopqrstuvwxyz{|}~", r, g, b, 0, 0, 0);
*/  
}
