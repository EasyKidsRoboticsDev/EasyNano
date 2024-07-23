#include "EasyNano.h"

void setup()
{
  EasyKids_Setup();
}

void loop()
{
  // Servo at pin 10, 11 & 12

  servo(10, 0);
  delay(1000);

  servo(10, 90);
  delay(1000);

  servo(10, 180);
  delay(1000);
}
