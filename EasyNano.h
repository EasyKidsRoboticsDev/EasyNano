#include <Adafruit_MCP3008.h>
#include <Servo.h>

#define PULLUP true
#define SPI_MODE 1
#define TWO_SENSORS_OUT 1 // Special case

#define DIR_1A 3
#define DIR_1B 4
#define PWM_1 5

#define DIR_2A 7
#define DIR_2B 8
#define PWM_2 6

#define START_SW 9

uint8_t NumSensor = 8;
bool invertedLine = false;
bool spi_mode = false;
bool twoSensorNoCenter = false; // Special case
 
uint8_t sensorPin[8] = { A0, A1, A2, A3, A4, A5, A6, A7 };
uint16_t Sensor_Min[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
uint16_t Sensor_Max[8] = { 1023, 1023, 1023, 1023, 1023, 1023, 1023, 1023 };

int setPoint, lastPosition = (NumSensor - 1) * 50;
float previous_error, error_sum = 0;

Adafruit_MCP3008 _adc;

void EasyKids_Setup()
{
  Serial.begin(9600);

  pinMode(DIR_1A, OUTPUT);
  pinMode(DIR_1B, OUTPUT);
  pinMode(PWM_1, OUTPUT);

  pinMode(DIR_2A, OUTPUT);
  pinMode(DIR_2B, OUTPUT);
  pinMode(PWM_2, OUTPUT);

  pinMode(START_SW, INPUT_PULLUP);

  // Stop motors
  analogWrite(PWM_1, 255);
  digitalWrite(DIR_1A, HIGH);
  digitalWrite(DIR_1B, HIGH);

  analogWrite(PWM_2, 255);
  digitalWrite(DIR_2A, HIGH);
  digitalWrite(DIR_2B, HIGH);
}

void waitForStart()
{
  while (digitalRead(START_SW)){}
}

bool sw_Start() 
{
  return digitalRead(START_SW);
}

void Motor_L(signed int speed)
{
  speed = map(speed, -100, 100, -255, 255);

  // Max speed = 255
  if (!speed)
  {
    analogWrite(PWM_1, 255);
    digitalWrite(DIR_1A, HIGH);
    digitalWrite(DIR_1B, HIGH);
  }
  else if(speed > 0)
  {
    speed = min(speed, 255);
    analogWrite(PWM_1, speed);
    digitalWrite(DIR_1A, LOW);
    digitalWrite(DIR_1B, HIGH);
  }
  else 
  {
    speed = abs(max(speed, -255));
    analogWrite(PWM_1, speed);
    digitalWrite(DIR_1A, HIGH);
    digitalWrite(DIR_1B, LOW);
  }
}

void Motor_R(signed int speed)
{
  speed = map(speed, -100, 100, -255, 255);

  // Max speed = 255
  if (!speed)
  {
    analogWrite(PWM_2, 255);
    digitalWrite(DIR_2A, HIGH);
    digitalWrite(DIR_2B, HIGH);
  }
  else if(speed > 0)
  {
    speed = min(speed, 255);
    analogWrite(PWM_2, speed);
    digitalWrite(DIR_2A, LOW);
    digitalWrite(DIR_2B, HIGH);
  }
  else 
  {
    speed = abs(max(speed, -255));
    analogWrite(PWM_2, speed);
    digitalWrite(DIR_2A, HIGH);
    digitalWrite(DIR_2B, LOW);
  }
}

void motor(uint8_t pin, int speed)
{
  if(pin == 1) { Motor_L(speed); }
  else if(pin == 2) { Motor_R(speed); }
}

void servo(uint8_t pin, int angle)
{
  angle = min(max(angle, 0), 180);
  Servo _servo;

  if(pin != 10 && pin != 11 && pin != 12) { return; }
  
  _servo.attach(pin);
  _servo.write(angle);
}

int analog(uint8_t pin)
{
  return pin < 8 ? analogRead(sensorPin[pin]) : 0;
}

bool digitalIn(uint8_t pin, bool pull_up = false)
{
  pull_up ? pinMode(pin, INPUT_PULLUP) : pinMode(pin, INPUT);
  return digitalRead(pin);
}

void digitalOut(uint8_t pin, uint8_t output)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, output);
}

void LineFollower_Setup(uint8_t mode = 0)
{
  EasyKids_Setup();
  spi_mode = mode;
  if(mode) { _adc.begin(); } // mode = USE_SPI (1)
}

int clamp(int val, int lowerLim, int upperLim)
{
  return min(max(val, lowerLim), upperLim);
}
void setSensorMin(int s0, int s1, int s2 = 0, int s3 = 0, int s4 = 0, int s5 = 0, int s6 = 0, int s7 = 0)
{
  Sensor_Min[0] = s0;
  Sensor_Min[1] = s1;
  Sensor_Min[2] = s2;
  Sensor_Min[3] = s3;
  Sensor_Min[4] = s4;
  Sensor_Min[5] = s5;
  Sensor_Min[6] = s6;
  Sensor_Min[7] = s7;
}

void setSensorMax(int s0, int s1, int s2 = 1023, int s3 = 1023, int s4 = 1023, int s5 = 1023, int s6 = 1023, int s7 = 1023)
{
  Sensor_Max[0] = s0;
  Sensor_Max[1] = s1;
  Sensor_Max[2] = s2;
  Sensor_Max[3] = s3;
  Sensor_Max[4] = s4;
  Sensor_Max[5] = s5;
  Sensor_Max[6] = s6;
  Sensor_Max[7] = s7;
}

void sensorNum(uint8_t num, uint8_t mode = 0)
{
  NumSensor = num;
  setPoint = (NumSensor - 1) * 50;
  lastPosition = setPoint;
  if(num == 2) { twoSensorNoCenter = mode; } // Only for two sensors (Middle gap required)
}

void whiteLine()
{
  invertedLine = true;
}
void blackLine()
{
  invertedLine = false;
}

int getPosition()
{
  bool onLine = false; // Check if robot still on line (Atleast 1 sensor)
  unsigned int avg = 0;
  unsigned int sum = 0;

  for (uint8_t i = 0; i < NumSensor; i++)
  {
    int value = spi_mode ? _adc.readADC(i) : analogRead(sensorPin[i]);
    value = clamp(map(value, Sensor_Min[i], Sensor_Max[i], 1, 100), 1, 100);
    if(invertedLine) { 101 - value; }  // White line

    if (value < 20)
    {
      onLine = true;
      avg += value * (i * 100);
      sum += value;
    }
  }

  if (!onLine)
  {
    if(twoSensorNoCenter)
    {
      lastPosition = 50;
    }
    else
    {
      lastPosition = (lastPosition >= setPoint) ? setPoint : 0;
    }
  }
  else
  {
    lastPosition = (avg / sum);
  }
  return round((setPoint * 2) - lastPosition);
}

void lineFollow(int min_speed, float kP, float kI, float kD)
{
  // Easier to adjust values
  kP /= 10; 
  kI /= 10; 
  kD /= 10;

  int current_error = getPosition() - setPoint;
  // Serial.println(getPostion());

  int output = (kP * current_error) + (kI * error_sum) + (kD * (current_error - previous_error));
  previous_error = current_error;
  error_sum += current_error;

  Motor_L(clamp(min_speed - output, -100, 100));
  Motor_R(clamp(min_speed + output, -100, 100));
}

void lineTimer(int min_speed, float kP, float kI, float kD, long setTime)
{
  long timeSince = 0;
  timeSince = millis();
  while ((millis() - timeSince) < setTime)
  {
    lineFollow(min_speed, kP, kI, kD);
  }
  Motor_L(0);
  Motor_R(0);
}

void lineCross(int min_speed, float kP, float kI, float kD)
{
  if(NumSensor < 6 && !twoSensorNoCenter) { return; }

  int rightVal = spi_mode ? _adc.readADC(0) : analogRead(sensorPin[0]);
  int leftVal = spi_mode ? _adc.readADC(NumSensor - 1) : analogRead(sensorPin[NumSensor - 1]);

  int avgRightVal = (Sensor_Max[0] + Sensor_Min[0]) / 2;
  int avgLeftVal = (Sensor_Max[NumSensor - 1] + Sensor_Min[NumSensor - 1]) / 2;

  while(true)
  {
    rightVal = spi_mode ? _adc.readADC(0) : analogRead(sensorPin[0]);
    leftVal =  spi_mode ? _adc.readADC(NumSensor - 1) : analogRead(sensorPin[NumSensor - 1]);
    lineFollow(min_speed, kP, kI, kD);

    if ((invertedLine && rightVal > avgRightVal || leftVal > avgLeftVal) || // White line
        (!invertedLine && rightVal < avgRightVal && leftVal < avgLeftVal)) // Black line
    { break; }
  }

  Motor_L(min_speed);
  Motor_R(min_speed);
  delay(map(min_speed, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void line90Left(int min_speed, float kP, float kI, float kD)
{
  if(NumSensor < 6 ) { return; }

  int leftVal = spi_mode ? _adc.readADC(2) : analogRead(sensorPin[2]);
  int lefterVal = spi_mode ? _adc.readADC(1) : analogRead(sensorPin[1]);
  int leftestVal = spi_mode ? _adc.readADC(0) : analogRead(sensorPin[0]);

  int avgLeftVal = (Sensor_Max[2] + Sensor_Min[2]) / 2;
  int avgLefterVal = (Sensor_Max[1] + Sensor_Min[1]) / 2;
  int avgLeftestVal = (Sensor_Max[0] + Sensor_Min[0]) / 2;

  while(true)
  {
    leftVal = spi_mode ? _adc.readADC(2) : analogRead(sensorPin[2]);
    lefterVal = spi_mode ? _adc.readADC(1) : analogRead(sensorPin[1]);
    leftestVal = spi_mode ? _adc.readADC(0) : analogRead(sensorPin[0]);
    lineFollow(min_speed, kP, kI, kD);

    if ((invertedLine && leftVal > avgLeftVal && lefterVal > avgLefterVal && leftestVal > avgLeftestVal) || // White line
        (!invertedLine && leftVal < avgLeftVal && lefterVal < avgLefterVal && leftestVal < avgLeftestVal)) // Black line
    { break; }
  }

  Motor_L(min_speed);
  Motor_R(min_speed);
  delay(map(min_speed, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void line90Right(int min_speed, float kP, float kI, float kD)
{
  if(NumSensor < 6 ) { return; }

  int rightVal = spi_mode ? _adc.readADC(NumSensor - 3) : analogRead(sensorPin[NumSensor - 3]);
  int righterVal = spi_mode ? _adc.readADC(NumSensor - 2) : analogRead(sensorPin[NumSensor - 2]);
  int rightestVal = spi_mode ? _adc.readADC(NumSensor - 1) : analogRead(sensorPin[NumSensor - 1]);

  int avgRightVal = (Sensor_Max[NumSensor - 3] + Sensor_Min[NumSensor - 3]) / 2;
  int avgRighterVal = (Sensor_Max[NumSensor - 2] + Sensor_Min[NumSensor - 2]) / 2;
  int avgRightestVal = (Sensor_Max[NumSensor - 1] + Sensor_Min[NumSensor - 1]) / 2;

  while(true)
  {
    rightVal = spi_mode ? _adc.readADC(NumSensor - 3) : analogRead(sensorPin[NumSensor - 3]);
    righterVal = spi_mode ? _adc.readADC(NumSensor - 2) : analogRead(sensorPin[NumSensor - 2]);
    rightestVal = spi_mode ? _adc.readADC(NumSensor - 1) : analogRead(sensorPin[NumSensor - 1]);
    lineFollow(min_speed, kP, kI, kD);

    if ((invertedLine && rightVal > avgRightVal && righterVal > avgRighterVal && rightestVal > avgRightestVal) || // White line
        (!invertedLine && rightVal < avgRightVal && righterVal < avgRighterVal && rightestVal < avgRightestVal)) // Black line
    { break; }
  }

  Motor_L(min_speed);
  Motor_R(min_speed);
  delay(map(min_speed, 0, 100, 100, 0));

  Motor_L(0);
  Motor_R(0);
}

void lineTurnLeft(int speedM)
{
  if(NumSensor < 6 ) { return; }

  Motor_L(-speedM);
  Motor_R(speedM);
  delay(70);

  do 
  {
    int sensorValue = spi_mode ? _adc.readADC(0) : analogRead(sensorPin[0]);

    Motor_L(-speedM);
    Motor_R(speedM);

    if((invertedLine && (sensorValue > (Sensor_Max[0] + Sensor_Min[0]) / 2)) || 
       (!invertedLine && (sensorValue < (Sensor_Max[0] + Sensor_Min[0]) / 2)))
    { break; }
  } while(true);
  
  Motor_L(0);
  Motor_R(0);
}

void lineTurnRight(int speedM)
{
  if(NumSensor < 6 ) { return; }

  Motor_L(speedM);
  Motor_R(-speedM);
  delay(70);

 do 
  {
    int sensorValue = spi_mode ? _adc.readADC(NumSensor - 1) : analogRead(sensorPin[NumSensor - 1]);

    Motor_L(-speedM);
    Motor_R(speedM);

    if((invertedLine && (sensorValue > (Sensor_Max[NumSensor - 1] + Sensor_Min[NumSensor - 1]) / 2)) || 
       (!invertedLine && (sensorValue < (Sensor_Max[NumSensor - 1] + Sensor_Min[NumSensor - 1]) / 2)))
    { break; }
  } while(true);

  Motor_L(0);
  Motor_R(0);
}

void readSensor(unsigned int time_delay = 100)
{
  while (1)
  {
    for (uint8_t i = 0; i < NumSensor; i++)
    {
      int sensorVal = spi_mode ? _adc.readADC(i) : analogRead(sensorPin[i]);
      Serial.print(sensorVal);
      Serial.print(" ");
    }
    Serial.println(" ");
    delay(time_delay);
  }
}
