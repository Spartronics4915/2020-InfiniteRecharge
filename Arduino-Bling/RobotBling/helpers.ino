/**
 * Helper methods
 */

// Set strip to full brightness
void setFullBrightness()
{
  pixels.setBrightness(255);
  pixels.show();
}

// Set strip to default brightness
void setDefaultBrightness()
{
  pixels.setBrightness(defaultSystemBrightness);
  pixels.show();
}

// Display pixels
void showPixels()
{
  pixels.show();
}

// Clear all pixels
void setAllPixelsOff()
{
  pixels.clear();
  pixels.show();
}

/**
 * Set all pixels to specified color
 * - Color is specified as r, g, b values
 * - Color is specifed as rgbColor value
 */
void setAllPixels(uint8_t r, uint8_t b, uint8_t g)
{
  pixels.fill(pixels.Color(r, b, g));
}

void setAllPixels(rgbColor c)
{
  // Reuse the existing method for setting pixel colors
  pixels.fill(pixels.Color(c.r, c.g, c.b));
}

/**
 * Set individual pixel values
 * - Color is specified as uint32_t color value
 * - Color specified as r,g,b values
 * paramenter 'i' refers to the pixel number
 */
void setPixel(uint8_t i, rgbColor c)
{
  pixels.setPixelColor(i, c.color);
}

void setPixel(uint8_t i, uint32_t color)
{
  pixels.setPixelColor(i, color);
}

void setPixel(uint8_t i, uint8_t r, uint8_t g, uint8_t b)
{
  pixels.setPixelColor(i, r, g, b);
}
