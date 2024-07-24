#include <EasyNano.h>

/*
  <<<<< Command >>>>>
  readSensor(time_delay); // default: 100ms
*/

void setup()
{
  LineFollower_Setup();
  sensorNum(8); // 2-8 sensors
  blackLine();
}

void loop()
{
  readSensor(200); // Show sensor value via serial monitor with 200ms delay
}