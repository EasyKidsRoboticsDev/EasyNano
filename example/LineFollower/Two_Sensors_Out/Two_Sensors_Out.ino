
#include <EasyNano.h>

/*
  <<<<< Command >>>>>
  sensorNum(2, TWO_SENSORS_NO_CENTER);
  lineFollow(speed, KP, KI, KD);
  lineTimer(speed, KP, KI, KD, Time(ms));
  lineCross(speed, KP, KI, KD);

  This is a special case and its purpose is mainly for competitions.
  Instead of having 2 sensors on the line, both are put on the left and right
  OUTSIDE of the line. This behaviour will allow the robot to go slightly faster 
  than the normal confirguration where both sensors are on the line. As a bonus, 
  we can also check for cross sections with this configuration.

  This is because of how the error position is calculated when the sensors are
  outside of the line. Normally it will return the lastest position before it went 
  off the track, but with this configuration we forcibly set the position to the 
  setpoint because the sensors aren't on the line anyways so we set it to the value
  it would be if the robot was centered on the line allowing it to move forward faster
  and not try to perform it's normal PID behaviour.
  
*/

void setup()
{
  LineFollower_Setup();

  sensorNum(2, TWO_SENSORS_OUT); 
  blackLine();

  setSensorMin(0, 0);
  setSensorMax(1023, 1023);
}

void loop()
{
  // readSensor();  //Show value 8 Sensor via Serial monitor
  waitForStart();

  lineTimer(40, 1.2, 0.0001, 1.0, 10000);    // Line follow for 10 seconds                             
  lineCross(40, 1.2, 0.0001, 1.0);           // Line follow until cross section                           
  lineTimer(40, 1.2, 0.0001, 1.0, 10000);    // Line follow for 10 seconds       
}
