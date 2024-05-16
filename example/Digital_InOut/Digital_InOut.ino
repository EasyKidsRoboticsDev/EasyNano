#include "EasyNano.h"

void setup()
{
  EasyKids_Setup();
}

void loop()
{
  Serial.print(digitalIn(11)); // Normal input
  Serial.print(" ");
  Serial.println(digitalIn_Pullup(9)); // Pullup_input
  digitalOut(2, HIGH);
}
