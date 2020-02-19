/*
   Spartronics Arduino Robot Bling Integration

   Arduino code for animation support:
   - Uses rgbColor struct to break color into its components for easy reference
   - Listens on serial port for new animation commands
   - ALL animations require call to `delay()` tp listen for interruptions

   Important:
   - Expected commands: ascii printable characters 0 and above
   - For development, start Arduino's serial port to test commands and animations
*/

/*
   Used by the magic _delay() function for running animations
*/
#include <setjmp.h>

/*
   Setup Adafruit NeoMatrix
   - Important: validate PIN connection and number of LEDs
   - Note: this NeoMatrix does not have 'w' component
*/
#include <Adafruit_NeoPixel.h>

// Configuration for our LED strip and Arduino
// TODO: Update PIN and NUM_LEDS for development and later robot deployment
#define NUM_LEDS 150   // was 30
#define PIN 11         // was 4

// Allocate our pixel memory, set interface to match our hardware
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);

// Save the desired system wide brightness value -- 0:255 --> level 32 == 1/8th the brightness
uint8_t defaultSystemBrightness = 32;

/*
   Animations require access to color information: both full color, or
   its rgb parts This rgbColor struct provides an easy access via the
   magic of 'union'
    - `union` allows storing different data types in the same memory location
    - Different rgbColor constructors enables easy parsing of data
   Reminder: we don't have 'w' component in our current NeoPixels. This code
   is not intended to be compatible with 4-component NeoPixels.
*/
struct rgbColor
{
  // initializers
  rgbColor() {
    color = 0x000000;  // if 'w' component was present: 0x00000000
  }
  rgbColor(uint32_t c) : color(c) {}
  rgbColor(uint8_t red, uint8_t green, uint8_t blue) : b(blue), g(green), r(red) {}

  union {
    uint32_t color; // full color
    struct
    {
      uint8_t b; // blue component
      uint8_t g; // green component
      uint8_t r; // red component
    };
  };
};

// Colors used in the animations
const rgbColor rgbColor_OFF(0, 0, 0);
const rgbColor rgbColor_RED(255, 0, 0);
const rgbColor rgbColor_GREEN(0x00ff00);
const rgbColor rgbColor_BLUE(0, 0, 255);
const rgbColor rgbColor_YELLOW(0xffff00);
const rgbColor rgbColor_ORANGE(255, 64, 0);
const rgbColor rgbColor_WHITE(0xffffff);
const rgbColor rgbColor_MAGENTA(0xff00ff);
const rgbColor rgbColor_MAGENTA_DIM(0x330033);

// Time in milliseconds for the flash of light
#define FLASH_TIME_INTERVAL     250

/*
   Animation index
    - List of animation used by Spartronics -- MUST match LED subsystem BlingStates
    - loop() matches the animation to the command entered on the serial port
*/
enum
{
  //NEEDS
  BLING_COMMAND_OFF = 0,
  BLING_COMMAND_STARTUP,
  BLING_COMMAND_DISABLED,
  //WANTS
  BLING_COMMAND_AUTOMODE,
  BLING_COMMAND_SHOOTING,
  BLING_COMMAND_PICKUP,
  BLING_COMMAND_LOADING,
  BLING_COMMAND_CLIMBING,
  BLING_COMMAND_VISION,
  //NEED
  BLING_COMMAND_DEFAULT,

  // Add new bling states immediately above this line

  ANIMATION_COUNT // The number of animations
};

/*
   Magic delay function for driving animation library
   - Note that the delay() function is overridden by the #define macro below
   - Anytime there is a delay() included in an animation, it allows to check for new commands
   - Commands drive the animation demo -- if a new command detected, we jump to the beginning
   of the loop -- see loop()

   Important: _delay() is listening for printable characters '0' and above.
   This matches the message sent by robot's LED subsystem
*/

// A global variable to hold the current command to be executed
uint8_t currentCommand = BLING_COMMAND_OFF;

// The breadcrumb to get us back to the beginning of the loop
// See 'man setjmp' for details on usage
jmp_buf env;

// Our magic delay function
// Repeat this loop until the requested time has passed
void _delay(uint16_t timeout)
{
  while (timeout != 0)
  {
    delay(1);
    timeout--;

    // If there are any characters available on the serial port, read them
    while (Serial.available() > 0)
    {
      char commandCharacter;
      commandCharacter = Serial.read();
      // Use the ASCII table to identify the >= commandCharacter order
      if (isprint(commandCharacter) && (commandCharacter >= '0'))
      {
        uint8_t command = commandCharacter - '0';
        if (command < ANIMATION_COUNT)
        {
          // Found a valid command!
          if (command != currentCommand)
          {
            // It's a new command!
            currentCommand = command;
            // Jump back to the top of the loop
            longjmp(env, 1);
          }
        }
      }
    }
  }
}

// Overriding delay function
#define delay _delay

/*
   Initialize system state, set up serial communications, and clear the LED strip
*/
void setup()
{
  Serial.begin(9600); // Required for listening to new commands

  // Initialize all pixels to 'off' with default brightness
  pixels.begin();
  // Set pixel brightness
  setDefaultBrightness();
  pixels.clear();
  pixels.show();
}

/*
   In our loop:
   - Setup breadcrumb for magic _delay()
   - Add animations to switch statement --> ensure matches the animation library above
*/
void loop()
{
  // Save a breadcrumb of where to jump back to
  if (setjmp(env) != 0)
  {
    // If we jumped back (non-zero return value), then exit the loop()
    // And, it will be re-run again, leaving a new breadcrumb.
    return;
  }

  // Clear the strip between animation changes
  pixels.clear();
  setDefaultBrightness();
  pixels.show();

  // From here everything should be interruptable (to switch animations)
  // as long as the animation code uses the `delay()` function.

  // Take the current command and set the parameters, i.e. animations
  switch (currentCommand)
  {
    // Annimations -- commands to match the enums above!

    //NEEDS BELOW

    case BLING_COMMAND_OFF: /* 0 */
      //Done
      solid(rgbColor_OFF);
      break;

    case BLING_COMMAND_STARTUP: /* 1 */
      //Done: was: strobe(rgbColor_ORANGE, 1, 100, 0);
      cogs(rgbColor_BLUE, rgbColor_YELLOW);
      break;

    case BLING_COMMAND_DISABLED: /* 2 */
      //Done
      solid(rgbColor_ORANGE);     // IMPORTANT: matching the RSL state @ disabled
      break;
    //Will adjust accordingly to the usual past robots blinking rate etc etc


    //WANTS BELOW
    case BLING_COMMAND_AUTOMODE: /* 3 */
      //Done
      solid(rgbColor_WHITE);
      break;

    case BLING_COMMAND_SHOOTING:  /* 4 */
      //Done but delay may need a retouch
      // was: strobe(rgbColor_WHITE, 1 /* # of flashes */, 150 /* flash delay */, 1000 /* strobe pause */);
      flash(FLASH_TIME_INTERVAL, rgbColor_WHITE.color, 255);
      currentCommand = BLING_COMMAND_OFF;       // since we may want to flash more than once, we are resetting command to OFF, so launch can rerun
     break;

    case BLING_COMMAND_PICKUP:  /* 5 */
      //Done --> was:  strobe(rgbColor_YELLOW, 1, 100, 0);
      spartronicsCrawler(10, rgbColor_BLUE.color, rgbColor_YELLOW.color, 30);
      break;

    case BLING_COMMAND_LOADING: /* 6 */
      //Done --> was: fillPixelByPixel (rgbColor_YELLOW, 50);
      fillSpartronicsColorsPixelByPixel(rgbColor_BLUE, rgbColor_YELLOW, 25);
      break;

    case BLING_COMMAND_CLIMBING:  /* 7 */
      //In-progress == suggestion: spartronics_fade()
      //was: solid(rgbColor_WHITE);
      spartronics_fade(10 /* wait */, rgbColor_BLUE.color /* color1 */, rgbColor_YELLOW.color /* color2 */, 255 /* brightness*/);
      break;

    case BLING_COMMAND_VISION: /* 8 */
      //Done
      strobe(rgbColor_WHITE, 1, 150, 500);
      break;

    case BLING_COMMAND_DEFAULT: /* 9 */
      //Done - was: solid(rgbColor_BLUE);
      cogs(rgbColor_BLUE, rgbColor_YELLOW);
      break;

    default:
      solid(rgbColor_OFF);
      // The currentCommand is not handled in this switch statement, and may be corrupt. Reset it to a good value.
      currentCommand = BLING_COMMAND_OFF;
  }
}
