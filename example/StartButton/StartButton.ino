#include "EasyNano.h"

void setup()
{
  EasyKids_Setup();
  waitForStart();  // Press start button first
}

void loop()
{
  Serial.println(sw_Start());
}
