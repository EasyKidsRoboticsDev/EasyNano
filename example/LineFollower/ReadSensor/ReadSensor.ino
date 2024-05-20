#include <EasyNano.h>

void setup()
{
  EasyKids_Setup();
  sensorNum(8); // Only 2, 7 & 8 available
  blackLine();
}

void loop()
{
  readSensor(); // Show sensor value via serial monitor
}