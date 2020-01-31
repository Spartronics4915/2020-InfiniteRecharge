/*
 * Robot animations
 */

/*
 * Solid colors - remember to add 'delay()' to listen for new commands
 */
void solid(rgbColor c)
{
  setAllPixels(c.color);
  showPixels();
  delay(10);
}

/**
 * Set strip to full brightness and display the solid color
 */
void solidWithFullBrightness(rgbColor c, uint8_t brightness)
{
  setFullBrightness();
  solid(c);
}

/**
 * Display solid color with 'off' in between to give illusion of 'blink'
 */
void solidWithBlink(rgbColor c, uint8_t wait)
{
  solid(c);
  blinkOffWithDelay(wait);
}

/*
 * Turns off the pixels creating a 'blink' animation
 */
void blinkOffWithDelay(uint8_t wait)
{
  // Wait prior to turning off pixels
  delay(wait);
  setAllPixelsOff();
  delay(wait);
}
