
#include <EasyNano.h>

/*
  <<<<< Command >>>>>

  LineFollower_Setup(); // Sensor analog interface
  LineFollower_Setup(SPI_MODE); // Sensor SPI interface

  sensorNum(8); // 2-8 sensors

  // Example: 2 Sensors
  sensorNum(2);
  setSensorMin(s0, s1);
  setSensorMax(s0, s1);

  // Example: 8 Sensors
  sensorNum(8);
  setSensorMin(s0, s1, s2, s3, s4, s5, s6, s7);
  setSensorMax(s0, s1, s2, s3, s4, s5, s6, s7);

  blackLine();
  whiteLine();

  readSensor();

  motor(pin, Speed); (-100 to 100) // Only 1 & 2 available

  // For any amount of sensors (2-8)
  lineFollow(speed, KP, KI, KD);
  lineTimer(speed, KP, KI, KD, Time(ms));

  // 6 or more sensors only due to constraints
  lineCross(speed, KP, KI, KD);
  line90Left(speed, KP, KI, KD);
  line90Right(speed, KP, KI, KD);
  lineTurnLeft(speed);
  lineTurnRight(speed);
*/

void setup()
{
  LineFollower_Setup();

  sensorNum(8); // Only 2, 7 & 8 available
  blackLine();

  setSensorMin(0, 0, 0, 0, 0, 0, 0, 0);
  setSensorMax(1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023);
}

void loop()
{
  // readSensor();  //Show value 8 Sensor via Serial monitor
  waitForStart();

  lineTimer(40, 1.2, 0.0001, 1.0, 5000);            // Line follow for 5 seconds
  lineTurnRight(30);                                // Turn right
  lineCross(40, 1.2, 0.0001, 1.0);                  // Line follow until cross section
  lineTurnLeft(30);                                 // Turn left
  line90Left(40, 1.2, 0.0001, 1.0);                 // Line follow until 90 degree section
}
