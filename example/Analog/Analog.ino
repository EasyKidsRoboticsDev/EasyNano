#include "EasyNano.h"

void setup()
{
  EasyKids_Setup();
}

void loop()
{
  Serial.println(analog(0));
}
