
#include <EasyNano.h>

/*
  <<<<< Command >>>>>

  EasyKids_Setup();

  sensorNum(7); // Only 2 & 7 available

  // Example: 2 Sensors
  sensorNum(2);
  setSensorMin(s0, s1);
  setSensorMax(s0, s1);

  // Example: 7 Sensors
  sensorNum(7);
  setSensorMin(s0, s1, s2, s3, s4, s5, s6);
  setSensorMax(s0, s1, s2, s3, s4, s5, s6);

  blackLine();
  whiteLine();

  readSensor();

  motor(pin, Speed); (-100 to 100) // Only 1 & 2 available

  lineFollow(speed, KP, KD);
  lineTimer(speed, KP, KD, Time(ms));
  lineCross(speed, KP, KD);
  lineFork(speed, KP, KD);
  line90Left(speed, KP, KD);
  line90Right(speed, KP, KD);
  lineTurnLeft(speed);
  lineTurnRight(speed);
*/

void setup()
{
  EasyKids_Setup();

  sensorNum(7); // Only 2 & 7 available
  blackLine();

  setSensorMin(0, 0, 0, 0, 0, 0, 0);
  setSensorMax(1023, 1023, 1023, 1023, 1023, 1023, 1023);
}

void loop()
{
  // readSensor();  //Show Value 7 Sensor via LCD Display

  waitForStart();

  lineTimer(40, 1.2, 1.0, 5000);
  lineTurnRight(30);
  lineCross(40, 1.2, 1.0);
  lineTurnLeft(30);
  line90Left(40, 1.2, 1.0);
}
