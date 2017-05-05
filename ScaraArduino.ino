/*
  SCARA ARM v0.2 - Aruidno Prototype  
  Sebastian Pendola
  New York - 2017
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// Create one motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Create objects for one servo motor and two stepper motors
Servo servo1;
Adafruit_StepperMotor *motor1 = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *motor2 = AFMS.getStepper(200, 2);

float a = 13.0;
float a2 = a * a;
float b = 12.35;
float b2 = b * b;
float ab2 = 2.0 * a * b;
float lowerArmAngle = 0;
float upperArmAngle = 0;
float lowerArmStepsPerDegree = 1.8;
float upperArmStepsPerDegree = 1.8;
float RadToDegree = 180.0/3.14159265359;

int motor1Counter = 0;
int motor2Counter = 0;
int lowerArmLimit = 53;
int upperArmLimit = 52;
int delayPerStep = 8;
int stepsPerLoop = 1;
int minMotorVelocity = 25;
int maxMotorVelocity = 40;
int lowerServo = 45;
int upperServo = 10;

void setup() 
{
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  delay(2000);
  Serial.println("ScaraArm - v0.2");

  // Switch Limits
  pinMode(lowerArmLimit, INPUT);
  pinMode(upperArmLimit, INPUT);

  // Default frequency 1.6Khz
  AFMS.begin();

  // Configure motors
  //servo1.attach(10);
  motor1->setSpeed(maxMotorVelocity);
  motor2->setSpeed(maxMotorVelocity);
  
  Handshake("Ready");
  ReferenceArm();
}

bool Handshake(String msg)
{
  char r = ' ';
  while(true)
  {
    if(!Serial.available())
    {
      Serial.println(msg);
      delay(1000);
    }
    while(Serial.available())
    {
      r = Serial.read();
      delay(1);
      if(r == 'k')
      {
        // Clear Buffer
        ClearBuffer();
        return true;
      }
    }
  }
}

void Blink(int count)
{
  for(int i=0; i<count; i++)
  {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(500);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    delay(500);              // wait for a second
  }
}

void ClearBuffer()
{
  while(Serial.available())
  {
    Serial.read();
    delay(1);
  }
}

char WaitForMessage(char* inBytes)
{
  int i = 0;
  while(Serial.available())
  {
    inBytes[i++] = Serial.read();
    delay(10);
  }
}

bool ReadLimit(int limitPin)
{
  digitalWrite(limitPin, HIGH);
  if(digitalRead(limitPin) == LOW)
    return true;
  return false;
}

bool ReferenceArm()
{
  Serial.println("Referencing Arm");
  
  // Retract the arm
  //MoveServo(upperServo);
  
  // Move lower and upper arm towards limits at normal speed
  MoveArmEx(1000, FORWARD, 1000, BACKWARD);
  delay(1000);

  // Move lower and upper arm away from limits
  MoveArmEx(30, BACKWARD, 30, FORWARD, true);
  delay(500);

  // Store maximum motor velocity
  int temp = maxMotorVelocity;
  maxMotorVelocity = minMotorVelocity;
  
  // Move lower and upper arm towards limits at minimal speed
  MoveArmEx(1000, FORWARD, 1000, BACKWARD);
  delay(500);

  // Restore max velocity
  maxMotorVelocity = temp;
  
  // Move away from limits reference distance
  MoveArmEx(15, BACKWARD, 130, FORWARD, true);
  delay(250);

  //MoveServo(lowerServo);
  upperArmAngle = 90;
  lowerArmAngle = 0;
  motor1Counter = 0;
  motor2Counter = 0;
  Handshake("Referenced");
  delay(1000);
}

void MoveArmEx(int lowerArmSteps, uint8_t lowerArmDirection, int upperArmSteps, uint8_t upperArmDirection)
{
  MoveArmEx(lowerArmSteps, lowerArmDirection, upperArmSteps, upperArmDirection, false);
}

void MoveArmEx(int lowerArmSteps, uint8_t lowerArmDirection, int upperArmSteps, uint8_t upperArmDirection, bool ignoreLimits)
{
  int lowerDistance = lowerArmSteps;
  int upperDistance = upperArmSteps;

  for(int i=0; i<lowerDistance; i+=stepsPerLoop)
  {
    motor1->step(stepsPerLoop, lowerArmDirection, DOUBLE);
    if(ReadLimit(lowerArmLimit) == true && !ignoreLimits)
        break;
    delay(delayPerStep);
  }

  for(int i=0; i<upperDistance; i+=stepsPerLoop)
  {
    motor2->step(stepsPerLoop, upperArmDirection, DOUBLE);
    if(ReadLimit(upperArmLimit) == true && !ignoreLimits)
        break;
    delay(delayPerStep);
  }
 
  motor1Counter = lowerArmDirection == BACKWARD ? motor1Counter + lowerArmSteps : motor1Counter - lowerArmSteps;
  motor2Counter = upperArmDirection == BACKWARD ? motor2Counter - upperArmSteps : motor2Counter + upperArmSteps;
}

void MoveArmToAngle(float newLowerArmAngle, float newUpperArmAngle)
{
  float lowerArm = (newLowerArmAngle - lowerArmAngle);
  float upperArm = (newUpperArmAngle - upperArmAngle);
  uint8_t lowerArmDirection = lowerArm >= 0 ? BACKWARD : FORWARD;
  uint8_t upperArmDirection = upperArm >= 0 ? FORWARD : BACKWARD;

  /*
  Serial.print("Moving Arm to Angles ");
  Serial.print(newLowerArmAngle);
  Serial.print(", ");
  Serial.print(newUpperArmAngle);
  Serial.print(" -> ");
  Serial.print(lowerArm);
  Serial.print(", ");
  Serial.println(upperArm);
  */
  
  MoveArmEx((int)(abs(lowerArm * lowerArmStepsPerDegree)), lowerArmDirection, (int)(abs(upperArm * upperArmStepsPerDegree)), upperArmDirection, false);
  lowerArmAngle = newLowerArmAngle;
  upperArmAngle = newUpperArmAngle;
}

void MoveArmToPosition(int x, int y)
{  
  // map correction
  y = y + 1;
  
  // Calculate cylindrical radius form center to x,y
  float r = sqrt((x*x) + (y*y));
  float tetha = acos(x/r)*RadToDegree;
  
  // Calculate UpperArm Angle
  float c2 = r*r;
  //Serial.print("Moving to Position: "); Serial.print(x); Serial.print(", "); Serial.println(y);
  float upperAngle = acos((a2+b2-c2)/(ab2))*RadToDegree;

  // Calculate LowerArm Angle
  float lowerAngle = acos((a2+c2-b2)/(2*b*r))*RadToDegree;
  MoveArmToAngle(180.0-(lowerAngle+tetha), upperAngle);
}


void loop() 
{
  char inBytes[3] = {'.', '.', '.'};
  WaitForMessage(inBytes);
  if(inBytes[0] != '.')
  {
    Handshake(String(inBytes[2]));
    MoveArmToPosition(inBytes[0] - 64, inBytes[1] - 64);
    Serial.println("c");
  }
    
}
