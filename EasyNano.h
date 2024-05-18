#define dir1A 3
#define dir1B 4
#define pwm1 5

#define dir2A 7
#define dir2B 8
#define pwm2 6

int NumSensor = 7;
bool invertedLine = false;

int sensorPin[7] = {A0, A1, A2, A3, A4, A5, A6};
int Sensor_Min[] = {0, 0, 0, 0, 0, 0, 0};
int Sensor_Max[] = {1023, 1023, 1023, 1023, 1023, 1023, 1023};

uint16_t setPoint = (NumSensor - 1) * 100 / 2;
uint16_t lastPosition = (NumSensor - 1) * 100 / 2;
float previous_error = (NumSensor - 1) * 100 / 2;

int output = 0;
int errors = 0;
int leftMotor = 0;
int rightMotor = 0;

void EasyKids_Setup()
{
  Serial.begin(9600);

  pinMode(dir1A, OUTPUT);
  pinMode(dir1B, OUTPUT);
  pinMode(pwm1, OUTPUT);

  pinMode(dir2A, OUTPUT);
  pinMode(dir2B, OUTPUT);
  pinMode(pwm2, OUTPUT);

  pinMode(9, INPUT_PULLUP);

  analogWrite(pwm1, 255);
  digitalWrite(dir1A, HIGH);
  digitalWrite(dir1B, HIGH);

  analogWrite(pwm2, 255);
  digitalWrite(dir2A, HIGH);
  digitalWrite(dir2B, HIGH);
}

int clamp(int val, int lowerLim, int upperLim)
{
  return min(max(val, lowerLim), upperLim);
}

void waitForStart()
{
  while (digitalRead(9))
  {
  }
}

void Motor_L(signed int speed)
{
  speed = map(speed, -100, 100, -255, 255);

  // Max speed = 255
  if (!speed)
  {
    analogWrite(pwm1, 255);
    digitalWrite(dir1A, HIGH);
    digitalWrite(dir1B, HIGH);
  }
  else if (speed > 0)
  {
    speed = min(speed, 255);

    analogWrite(pwm1, speed);
    digitalWrite(dir1A, LOW);
    digitalWrite(dir1B, HIGH);
  }
  else
  {
    speed = max(speed, -255);
    speed *= (-1);

    analogWrite(pwm1, speed);
    digitalWrite(dir1A, HIGH);
    digitalWrite(dir1B, LOW);
  }
}

void Motor_R(signed int speed)
{
  speed = map(speed, -100, 100, -255, 255);

  // Max speed = 255
  if (!speed)
  {
    analogWrite(pwm2, 255);
    digitalWrite(dir2A, HIGH);
    digitalWrite(dir2B, HIGH);
  }
  else if (speed > 0)
  {
    speed = min(speed, 255);

    analogWrite(pwm2, speed);
    digitalWrite(dir2A, LOW);
    digitalWrite(dir2B, HIGH);
  }
  else
  {
    speed = max(speed, -255);
    speed *= (-1);

    analogWrite(pwm2, speed);
    digitalWrite(dir2A, HIGH);
    digitalWrite(dir2B, LOW);
  }
}

void motor(uint8_t pin, int speed)
{
  if (pin == 1)
  {
    Motor_L(speed);
  }
  else if (pin == 2)
  {
    Motor_R(speed);
  }
}

int analog(uint8_t pin)
{
  if (pin == 0)
  {
    pin = A0;
  }
  else if (pin == 1)
  {
    pin = A1;
  }
  else if (pin == 2)
  {
    pin = A2;
  }
  else if (pin == 3)
  {
    pin = A3;
  }
  else if (pin == 4)
  {
    pin = A4;
  }
  else if (pin == 5)
  {
    pin = A5;
  }
  else if (pin == 6)
  {
    pin = A6;
  }
  else if (pin == 7)
  {
    pin = A7;
  }
  else
  {
    return 0;
  }
  return analogRead(pin);
}

int digitalIn(uint8_t pin)
{
  pinMode(pin, INPUT);
  return digitalRead(pin);
}

int digitalIn_Pullup(int pin)
{
  pinMode(pin, INPUT_PULLUP);
  return digitalRead(pin);
}

int sw_Start()
{
  return digitalRead(9);
}

void digitalOut(uint8_t pin, uint8_t output)
{
  pinMode(pin, OUTPUT);
  digitalWrite(pin, output);
}

void setSensorMin(int s0, int s1, int s2 = 0, int s3 = 0, int s4 = 0, int s5 = 0, int s6 = 0)
{
  Sensor_Min[0] = s0;
  Sensor_Min[1] = s1;
  Sensor_Min[2] = s2;
  Sensor_Min[3] = s3;
  Sensor_Min[4] = s4;
  Sensor_Min[5] = s5;
  Sensor_Min[6] = s6;
}

void setSensorMax(int s0, int s1, int s2 = 1023, int s3 = 1023, int s4 = 1023, int s5 = 1023, int s6 = 1023)
{
  Sensor_Max[0] = s0;
  Sensor_Max[1] = s1;
  Sensor_Max[2] = s2;
  Sensor_Max[3] = s3;
  Sensor_Max[4] = s4;
  Sensor_Max[5] = s5;
  Sensor_Max[6] = s6;
}

void sensorNum(int num)
{
  if (num != 2 && num != 7)
  {
    return;
  }
  else
  {
    NumSensor = num;
    setPoint = (NumSensor - 1) * 100 / 2;
    lastPosition = (NumSensor - 1) * 100 / 2;
    previous_error = (NumSensor - 1) * 100 / 2;
  }
}

void whiteLine()
{
  invertedLine = true;
}
void blackLine()
{
  invertedLine = false;
}

int getPostion()
{
  bool onLine = false; // Check if robot still on line (Atleast 1 sensor)
  unsigned int avg = 0;
  unsigned int sum = 0;

  for (int i = 0; i < NumSensor; i++)
  {
    int value = 0;

    value = clamp(map(analogRead(sensorPin[i]), Sensor_Min[i], Sensor_Max[i], 1, 100), 1, 100);
    if (invertedLine) // White line
    {
      value = 101 - value;
    }

    if (value < 20)
    {
      onLine = true;
      avg += value * (i * 100);
      sum += value;
    }

    // Serial.print(value);
    // Serial.print(" / ");
  }
  // Serial.println("");

  // Serial.print(avg);
  // Serial.print(" / ");
  //  Serial.print(sum);
  // Serial.print(" / ");
  //  Serial.print(avg / sum);
  // Serial.print(" / ");
  //  Serial.println(onLine);

  if (!onLine)
  {
    if (lastPosition >= ((NumSensor - 1) * 100) / 2)  // Black line
    {
      lastPosition = (NumSensor - 1) * 100; // Middle value
    }
    else
    {
      lastPosition = 0;
    }
  }
  else
  {
    lastPosition = (avg / sum);
  }
  return round(((NumSensor - 1) * 100) - lastPosition);
}

void lineFollow(int min_speed, float iKP, float iKD)
{

  iKP = iKP / 10;
  iKD = iKD / 10;

  errors = (getPostion() - setPoint);
  Serial.println(getPostion());
  output = (iKP * errors) + (iKD * (errors - previous_error));
  previous_error = errors;
  leftMotor = clamp(min_speed - output, -100, 100);
  rightMotor = clamp(min_speed + output, -100, 100);

  Motor_L(leftMotor);
  Motor_R(rightMotor);
}

void lineTimer(int speed, float iKP, float iKD, long setTime)
{

  long timeSince = 0;
  timeSince = millis();
  while ((millis() - timeSince) < setTime)
  {
    lineFollow(speed, iKP, iKD);
  }
  Motor_L(0);
  Motor_R(0);
}

void lineCross(int setSpeed, float iKP, float iKD)
{
  if (NumSensor < 7)
  {
    return;
  }

  int rightVal = analogRead(sensorPin[0]);
  int leftVal = analogRead(sensorPin[NumSensor - 1]);

  int avgRightVal = (Sensor_Max[0] + Sensor_Min[0]) / 2;
  int avgLeftVal = (Sensor_Max[NumSensor - 1] + Sensor_Min[NumSensor - 1]) / 2;

  if (invertedLine)
  {
    while (rightVal < avgRightVal || leftVal < avgLeftVal)
    {
      rightVal = analogRead(sensorPin[0]);
      leftVal = analogRead(sensorPin[NumSensor - 1]);
      lineFollow(setSpeed, iKP, iKD);
    }
  }
  else
  {
    while (rightVal > avgRightVal || leftVal > avgLeftVal)
    {
      rightVal = analogRead(sensorPin[0]);
      leftVal = analogRead(sensorPin[NumSensor - 1]);
      lineFollow(setSpeed, iKP, iKD);
    }
  }
  Motor_L(setSpeed);
  Motor_R(setSpeed);
  delay(map(setSpeed, 0, 100, 100, 0));
  Motor_L(0);
  Motor_R(0);
}

void lineFork(int setSpeed, float iKP, float iKD)
{
  if (NumSensor < 7)
  {
    return;
  }

  int outerRightVal = analogRead(sensorPin[0]);
  int outerLeftVal = analogRead(sensorPin[NumSensor - 1]);

  int avgOuterRightVal = (Sensor_Max[0] + Sensor_Min[0]) / 2;
  int avgOuterLeftVal = (Sensor_Max[NumSensor - 1] + Sensor_Min[NumSensor - 1]) / 2;

  if (invertedLine)
  {
    while (outerRightVal < avgOuterRightVal || outerLeftVal < avgOuterLeftVal)
    {
      outerRightVal = analogRead(sensorPin[0]);
      outerLeftVal = analogRead(sensorPin[NumSensor - 1]);

      lineFollow(setSpeed, iKP, iKD);
    }
  }
  else
  {
    while (outerRightVal > avgOuterRightVal || outerLeftVal > avgOuterLeftVal)
    {
      outerRightVal = analogRead(sensorPin[0]);
      outerLeftVal = analogRead(sensorPin[NumSensor - 1]);

      lineFollow(setSpeed, iKP, iKD);
    }
  }
  Motor_L(setSpeed);
  Motor_R(setSpeed);
  delay(map(setSpeed, 0, 100, 100, 0));
  Motor_L(0);
  Motor_R(0);
}

void line90Left(int setSpeed, float iKP, float iKD)
{
  if (NumSensor < 7)
  {
    return;
  }

  int midVal = analogRead(sensorPin[3]);
  int leftVal = analogRead(sensorPin[2]);
  int lefterVal = analogRead(sensorPin[1]);
  int leftestVal = analogRead(sensorPin[0]);

  int avgMidVal = (Sensor_Max[3] + Sensor_Min[3]) / 2;
  int avgLeftVal = (Sensor_Max[2] + Sensor_Min[2]) / 2;
  int avgLefterVal = (Sensor_Max[1] + Sensor_Min[1]) / 2;
  int avgLeftestVal = (Sensor_Max[0] + Sensor_Min[0]) / 2;

  if (invertedLine)
  {
    while (midVal < avgMidVal || leftVal < avgLeftVal || lefterVal < avgLefterVal || leftestVal < avgLeftestVal)
    {
      midVal = analogRead(sensorPin[3]);
      leftVal = analogRead(sensorPin[2]);
      lefterVal = analogRead(sensorPin[1]);
      leftestVal = analogRead(sensorPin[0]);
      lineFollow(setSpeed, iKP, iKD);
    }
  }
  else
  {
    while (midVal > avgMidVal || leftVal > avgLeftVal || lefterVal > avgLefterVal || leftestVal > avgLeftestVal)
    {
      midVal = analogRead(sensorPin[3]);
      leftVal = analogRead(sensorPin[2]);
      lefterVal = analogRead(sensorPin[1]);
      leftestVal = analogRead(sensorPin[0]);
      lineFollow(setSpeed, iKP, iKD);
    }
  }
  Motor_L(setSpeed);
  Motor_R(setSpeed);
  delay(map(setSpeed, 0, 100, 100, 0));
  Motor_L(0);
  Motor_R(0);
}

void line90Right(int setSpeed, float iKP, float iKD)
{
  if (NumSensor < 7)
  {
    return;
  }

  int midVal = analogRead(sensorPin[3]);
  int rightVal = analogRead(sensorPin[4]);
  int righterVal = analogRead(sensorPin[5]);
  int rightestVal = analogRead(sensorPin[6]);

  int avgMidVal = (Sensor_Max[3] + Sensor_Min[3]) / 2;
  int avgRightVal = (Sensor_Max[4] + Sensor_Min[4]) / 2;
  int avgRighterVal = (Sensor_Max[5] + Sensor_Min[5]) / 2;
  int avgRightestVal = (Sensor_Max[6] + Sensor_Min[6]) / 2;

  if (invertedLine)
  {
    while (midVal < avgMidVal || rightVal < avgRightVal || righterVal < avgRighterVal || rightestVal < avgRightestVal)
    {
      midVal = analogRead(sensorPin[3]);
      rightVal = analogRead(sensorPin[4]);
      righterVal = analogRead(sensorPin[5]);
      rightestVal = analogRead(sensorPin[6]);
      lineFollow(setSpeed, iKP, iKD);
    }
  }
  else
  {
    while (midVal > avgMidVal || rightVal > avgRightVal || righterVal > avgRighterVal || rightestVal > avgRightestVal)
    {
      midVal = analogRead(sensorPin[3]);
      rightVal = analogRead(sensorPin[4]);
      righterVal = analogRead(sensorPin[5]);
      rightestVal = analogRead(sensorPin[6]);
      lineFollow(setSpeed, iKP, iKD);
    }
  }
  Motor_L(setSpeed);
  Motor_R(setSpeed);
  delay(map(setSpeed, 0, 100, 100, 0));
  Motor_L(0);
  Motor_R(0);
}

void lineTurnLeft(int speedM)
{
  if (NumSensor < 7)
  {
    return;
  }

  // Motor_L(-speedM);
  // Motor_R(speedM);
  // delay(70);
  if (invertedLine)
  {
    do
    {
      Motor_L(-speedM);
      Motor_R(speedM);
    } while (analogRead(sensorPin[0]) < (Sensor_Max[0] + Sensor_Min[0]) / 2);
  }
  else
  {
    do
    {
      Motor_L(-speedM);
      Motor_R(speedM);
    } while (analogRead(sensorPin[0]) > (Sensor_Max[0] + Sensor_Min[0]) / 2);
  }
  Motor_L(0);
  Motor_R(0);
}

void lineTurnRight(int speedM)
{
  if (NumSensor < 7)
  {
    return;
  }

  // Motor_L(speedM);
  // Motor_R(-speedM);
  // delay(70);
  if (invertedLine)
  {
    do
    {
      Motor_L(speedM);
      Motor_R(-speedM);
    } while (analogRead(sensorPin[6]) < (Sensor_Max[6] + Sensor_Min[6]) / 2);
  }
  else
  {
    do
    {
      Motor_L(speedM);
      Motor_R(-speedM);
    } while (analogRead(sensorPin[6]) > (Sensor_Max[6] + Sensor_Min[6]) / 2);
  }
  Motor_L(0);
  Motor_R(0);
}

void readSensor()
{
  while (1)
  {
    // Serial
    Serial.print(analogRead(sensorPin[0]));
    Serial.print(", ");
    Serial.print(analogRead(sensorPin[1]));

    if (NumSensor == 7)
    {
      Serial.print(", ");
      Serial.print(analogRead(sensorPin[2]));
      Serial.print(", ");
      Serial.print(analogRead(sensorPin[3]));
      Serial.print(", ");
      Serial.print(analogRead(sensorPin[4]));
      Serial.print(", ");
      Serial.print(analogRead(sensorPin[5]));
      Serial.print(", ");
      Serial.print(analogRead(sensorPin[6]));
    }

    Serial.println(" ");
    delay(50);
  }
}