#include "EasyNano.h"

void setup()
{
  EasyKids_Setup();
}

void loop()
{
  // Only for motor 1 & 2

  motor(1, 50);
  motor(2, 50);
  delay(1000);

  motor(1, -50);
  motor(2, -50);
  delay(1000);

  motor(1, 0);
  motor(2, 0);
  delay(1000);
}
