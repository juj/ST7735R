// This sample shows how to maintain a "text box" UI element that is dynamically redrawn in a way that is free from flickering, i.e. instead of clearing the whole box before rendering,
// the new text is rendered, and after that the "tail" of the text area is cleared from any old text.

#ifndef ST7735R_MONACO_FONT
#error Please #define ST7735R_MONACO_FONT at the top of ST7735R.ino file!
#endif

const int textBoxX = 5;
const int textBoxY = 100;
const int textYAdjust = 1; // Add one pixel vertically since Truetype font baseline is one pixel up in the generated font data.
const int maxTextBoxWidthChars = 6;
const int letterWidthInPixels = 6;
const int letterHeightInPixels = 8;
const int textBoxWidthPixels = maxTextBoxWidthChars * letterWidthInPixels;
const int textBoxHeightPixels = letterHeightInPixels;

const int screenBackgroundR = 100;
const int screenBackgroundG = 100;
const int screenBackgroundB = 200;

const int textboxBackgroundR = 200;
const int textboxBackgroundG = 200;
const int textboxBackgroundB = 255;

void setup_test()
{
#ifdef USE_ARDUINO_TFT
  tft.setTextSize(1);
#endif

  // Clear the whole screen only once.
#ifdef USE_ARDUINO_TFT
  tft.background(screenBackgroundR, screenBackgroundG, screenBackgroundB);
#else
  ST7735R_FillRect(0, 0, ST7735R_WIDTH-1, ST7735R_HEIGHT-1, screenBackgroundR, screenBackgroundG, screenBackgroundB);
#endif

  const int leftMarginX = 1;
  const int topMarginY = 1;

  // Initialize the background color for the text box, only once.
#ifdef USE_ARDUINO_TFT
  tft.noStroke();
  tft.fill(textboxBackgroundR, textboxBackgroundG, textboxBackgroundB);
  tft.rect(textBoxX - leftMarginX, textBoxY - topMarginY, textBoxWidthPixels + leftMarginX, textBoxHeightPixels + topMarginY);
#else
  ST7735R_FillRect(textBoxX - leftMarginX, textBoxY - topMarginY, textBoxX + textBoxWidthPixels - 1, textBoxY + textBoxHeightPixels - 1, textboxBackgroundR, textboxBackgroundG, textboxBackgroundB);
#endif
}

void loop_test()
{
  unsigned int sensor_reading = 100 * (unsigned int)(cos(millis() / 1000.f) * 320 + 320);
  char text[22];
  sprintf(text, "%u", sensor_reading);
  
  const int maxChars = 8;
  int numChars = strlen(text);

  // Draw the new text (this will clear the old.
#ifdef USE_ARDUINO_TFT
  tft.setTextColor(tft.newColor(0, 0, 0), tft.newColor(textboxBackgroundR, textboxBackgroundG, textboxBackgroundB));
  tft.setCursor(textBoxX, textBoxY);
  tft.print(text); 
#else
  ST7735R_DrawText(textBoxX, textBoxY + textYAdjust, text, 0, 0, 0, textboxBackgroundR, textboxBackgroundG, textboxBackgroundB);
#endif

  // Clear the tail. (a slightly more advanced version could remember how wide the previously rendered content was, and how much tail is needed to be cleared. Not shown here.)
#ifdef USE_ARDUINO_TFT
  tft.noStroke();
  tft.fill(textboxBackgroundR, textboxBackgroundG, textboxBackgroundB);
  tft.rect(textBoxX + numChars*6, textBoxY, textBoxWidthPixels - numChars*6, textBoxHeightPixels);
#else
  ST7735R_FillRect(textBoxX + numChars*6, textBoxY, textBoxX + textBoxWidthPixels - 1, textBoxY + textBoxHeightPixels - 1, textboxBackgroundR, textboxBackgroundG, textboxBackgroundB);
#endif
}

