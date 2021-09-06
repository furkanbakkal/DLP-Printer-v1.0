// Spiro
// Rainbow patern generator
// Alan Senior 22/2/15

#include "SPI.h"
#include "Adafruit_GFX_AS.h"
#include "Adafruit_ILI9341_AS.h"

#define sclk 13  // Don't change
#define mosi 11  // Don't change
#define cs   10
#define dc   9
#define rst  7  // you can also connect this to the Arduino reset

#define ILI9341_GREY 0x7BEF
Adafruit_ILI9341_AS tft = Adafruit_ILI9341_AS(cs, dc, rst);       // Invoke custom library

unsigned long runTime = 0;

float sx = 0, sy = 0;
uint16_t x0 = 0, x1 = 0, y0 = 0, y1 = 0;

void setup()
{
  randomSeed(analogRead(A0));
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  delay(10);
  digitalWrite(7, HIGH);
  // Setup the LCD
  tft.init();
  tft.setRotation(3);
}

void loop()
{
  runTime = millis();

  tft.fillScreen(ILI9341_BLACK);
  int n = random(2, 19), r = random(20, 100), colour = 0; //rainbow();
  
  for (long i = 0; i < (360 * n); i++) {
    sx = cos((i / n - 90) * 0.0174532925);
    sy = sin((i / n - 90) * 0.0174532925);
    x0 = sx * (120 - r) + 159;
    y0 = sy * (120 - r) + 119;


    sy = cos(((i % 360) - 90) * 0.0174532925);
    sx = sin(((i % 360) - 90) * 0.0174532925);
    x1 = sx * r + x0;
    y1 = sy * r + y0;
    tft.drawPixel(x1, y1, rainbow(i % 128)); //colour);
  }
  
  r = random(20, 100);//r = r / random(2,4);
  for (long i = 0; i < (360 * n); i++) {
    sx = cos((i / n - 90) * 0.0174532925);
    sy = sin((i / n - 90) * 0.0174532925);
    x0 = sx * (120 - r) + 159;
    y0 = sy * (120 - r) + 119;


    sy = cos(((i % 360) - 90) * 0.0174532925);
    sx = sin(((i % 360) - 90) * 0.0174532925);
    x1 = sx * r + x0;
    y1 = sy * r + y0;
    tft.drawPixel(x1, y1, rainbow(i % 128)); //colour);
  }


  delay(2000);
}

unsigned int rainbow(int value)
{
  // Value is expected to be in range 0-127
  // The value is converted to a spectrum colour from 0 = blue through to red = blue
  //int value = random (128);
  byte red = 0; // Red is the top 5 bits of a 16 bit colour value
  byte green = 0;// Green is the middle 6 bits
  byte blue = 0; // Blue is the bottom 5 bits

  byte quadrant = value / 32;

  if (quadrant == 0) {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1) {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2) {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3) {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}


