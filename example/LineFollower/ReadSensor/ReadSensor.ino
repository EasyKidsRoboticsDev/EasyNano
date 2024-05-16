#include <EasyNano.h>

void setup()
{
  EasyKids_Setup();
  sensorNum(7); // Only 2 & 7 available
  blackLine();
}

void loop()
{
  readSensor(); // Show sensor value via serial monitor
}