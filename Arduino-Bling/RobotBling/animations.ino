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

 void strobe(rgbColor c, uint8_t numFlashes, int8_t flashDelay, int8_t strobePause)
 {
  for (int j = 0; j < numFlashes; j++) 
  {
    setAllPixels(c);
    showPixels();
    
    delay(flashDelay);
    
    setAllPixels(rgbColor_OFF);
    showPixels();
    delay(flashDelay);
  }
 
 delay(strobePause);
}

void fillPixelByPixel(rgbColor c, uint8_t wait)
 {
  // start w/ pixel 0, set color, show color, wait and do it again
  for(uint16_t i=0; i<pixels.numPixels(); i++) 
  {
    // pass the color component of rgbColor 
    setPixel(i, c.color);
    showPixels();

    delay(wait);
  }
}
