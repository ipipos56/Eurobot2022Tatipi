#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <Wire.h>
#include <Servo.h>
#include <EncButton.h>
#define EB_BETTER_ENC

Servo myservo12;
Servo myservo23;
Servo myservo31;
SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 10000);
DynamicJsonDocument doc(2048);

#define BNO055_SAMPLERATE_DELAY_MS (20)

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);


#define address1 0x80
#define address2 0x81

#define pi PI

#define R 0.058/2 * 100
#define _R 0.16 * 100
#define Radius 0.32 * 100
#define Kp 3.73525
#define Ki 0.56997
#define Kd 0.25
#define qpps 3600

uint32_t myTimer1;
int period = 20;

long int enc1;
long int enc2;
long int enc3;
int x = 0;
int xx = 0;
int correction = 0;
bool _finishMoving = 0;

float errA = 0;
float errB = 0;
float errC = 0;
float errEncA = 0;
float errEncB = 0;
float errEncC = 0;
float prevErrE*ncA = 0;
float prevErrEncB = 0;
float prevErrEncC = 0;
int Ua = 0;
int Ub = 0;
int Uc = 0;
int UEncA = 0;
int UEncB = 0;
int UEncC = 0;
float angleOfRobot = 0;
float prevErrA, prevErrB, prevErrC;
float integralA = 0, integralB = 0, integralC = 0;
float Da, Db, Dc;
float integralEncA = 0, integralEncB = 0, integralEncC = 0;
float DEncA, DEncB, DEncC;
float alpha = 0;
float Va = 0;
float Vb = 0;
float Vc = 0;
int Va_base = 0;
int Vb_base = 0;
int Vc_base = 0;
int velocity;
int aссel;
extern volatile unsigned long timer0_millis;

void CalculateSpeed(int _angle)
{
  Va_base = velocity * cos((150 * PI / 180 + _angle * PI / 180 - alpha));
  Vb_base = velocity * cos((30 * PI / 180 + _angle * PI / 180 - alpha));
  Vc_base = velocity * cos((270 * PI / 180 + _angle * PI / 180 - alpha));
  accel = millis();
}

void pidForMotor()
{
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  roboclaw.SpeedM2(address1, 0);
  int smn = 0;
  int _angle = 0;
  velocity = 2000;
  alpha = 10 * PI / 180;
  CalculateSpeed(0);
  while (1)
  {
    if (millis() - myTimer1 >= period) {   // ищем разницу (500 мс)
      myTimer1 += period;                  // сброс таймера
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      x = euler.x();
      _angle = normilize(_angle);
      smn = _angle - x;
      if (smn > 180)
        smn -= 360;
      else if (smn < -180)
        smn += 360;
      //Serial.println(x);

    }
    if ((abs(smn)) >= 2)
    {
      rotationWithPID(_angle);
    }
    kAc = min(1, (float)(millis() - accel) / 500)
    roboclaw.SpeedM1(address1,(int)(Va_base * kAc));
    roboclaw.SpeedM2(address2,(int)(Vb_base * kAc));
    roboclaw.SpeedM2(address1,(int)(Vc_base * kAc));
  }
}

void setup()
{
  Serial.begin(38400);
  roboclaw.begin(38400);
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
    {
      Serial.println("Reconnect");
    }
  }
  Serial.println("Hello");
  bno.setExtCrystalUse(true);
  timer0_millis = UINT32_MAX - 5000;
  roboclaw.SetM1VelocityPID(address1, Kd, Kp, Ki, qpps);
  roboclaw.SetM2VelocityPID(address2, Kd, Kp, Ki, qpps);
  roboclaw.SetM2VelocityPID(address1, Kd, Kp, Ki, qpps);
}

int normilize(int angle)
{
  angle = angle % 360;
  if (angle < 0)
    angle += 360;
  return angle;
}

bool rotationWithPID(int _angle)
{
  float rotationErr = 0, previousRotationErr = 0;
  float difRotation = 0, intRotation = 0, prRotation = 0;
  float kDifRotation = 6, kIntRotation = 0.2, kPrRotation = 3.5;
  float _dt = 10;
  int controlAct = 0;
  int smn = 0;
  while (1)
  {
    if (millis() - myTimer1 >= period)
    {
      myTimer1 += period;
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      x = euler.x();
      _angle = normilize(_angle);
      smn = _angle - x;
      if (smn > 180)
        smn -= 360;
      else if (smn < -180)
        smn += 360;
      //Serial.println(x);
    }
    rotationErr = smn;
    prRotation = rotationErr * kPrRotation;
    difRotation = ((rotationErr - previousRotationErr) / _dt) * kDifRotation;
    previousRotationErr = rotationErr;
    intRotation = (intRotation + rotationErr * _dt) * kIntRotation;
    if (rotationErr > 0)
      controlAct = (int)(prRotation + difRotation + intRotation);
    else
      controlAct = (int)(prRotation + difRotation + intRotation);
    if (rotationErr > 0)
    {
      controlAct = constrain(abs(controlAct), 0, 127);
      roboclaw.BackwardM1(address1, abs(controlAct));
      roboclaw.BackwardM2(address2, abs(controlAct));
      roboclaw.BackwardM2(address1, abs(controlAct));
    }
    else
    {
      controlAct = constrain(abs(controlAct), 0, 127);
      roboclaw.ForwardM1(address1, abs(controlAct));
      roboclaw.ForwardM2(address2, abs(controlAct));
      roboclaw.ForwardM2(address1, abs(controlAct));
    }
    if (abs(rotationErr) < 2)
      break;
  }
  roboclaw.ForwardM1(address1, 0);
  roboclaw.ForwardM2(address2, 0);
  roboclaw.ForwardM2(address1, 0);
}

void loop()
{
  pidForMotor();
  delay(1000);
  //rotationWithPID(-90);
  //rotationWithPID(0);
  //rotationWithPID(90);
  //rotationWithPID(0);
  while (1);
}
