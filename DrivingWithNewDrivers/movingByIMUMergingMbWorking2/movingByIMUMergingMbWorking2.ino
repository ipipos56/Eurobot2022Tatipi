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
float prevErrEncA = 0;
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
int Vmax = 0;
float Ma = 1;
float Mb = 1;
float Mc = 1;
float dt = 50;

int velocity = 20;
int EncMaxNum = 0;
int maxEnc = 0;
int tickA = 0;
int tickB = 0;
int tickC = 0;

extern volatile unsigned long timer0_millis;

void resetAll()
{
  roboclaw.SetEncM1(address1, 0);
  roboclaw.SetEncM2(address2, 0);
  roboclaw.SetEncM2(address1, 0);
  maxEnc = 0;
  errA = 0;
  errB = 0;
  errC = 0;
  errEncA = 0;
  errEncB = 0;
  errEncC = 0;
  prevErrEncA = 0;
  prevErrEncB = 0;
  prevErrEncC = 0;
  prevErrA = 0;
  prevErrB = 0;
  prevErrC = 0;
  Ua = 0;
  Ub = 0;
  Uc = 0;
  UEncA = 0;
  UEncB = 0;
  UEncC = 0;
  Da = 0;
  Db = 0;
  Dc = 0;
  DEncA = 0;
  DEncB = 0;
  DEncC = 0;
}

void CalculateSpeed(int _angle)
{
  Va_base = velocity * cos((150 * PI / 180 + _angle * PI / 180 - alpha));
  Vb_base = velocity * cos((30 * PI / 180 + _angle * PI / 180 - alpha));
  Vc_base = velocity * cos((270 * PI / 180 + _angle * PI / 180 - alpha));
}

void CalculateKoef()
{
  Vmax = 0;
  EncMaxNum = 0;
  if (abs(Va_base) > abs(Vmax))
  {
    Vmax = Va_base;
    EncMaxNum = 1;
  }
  if (abs(Vb_base) > abs(Vmax))
  {
    Vmax = Vb_base;
    EncMaxNum = 2;
  }
  if (abs(Vc_base) > abs(Vmax))
  {
    Vmax = Vc_base;
    EncMaxNum = 3;
  }
  if ((Va_base != 0) && (Vmax != 0))  Ma = Va_base / Vmax;
  else Ma = 0;
  if ((Vb_base != 0) && (Vmax != 0))  Mb = Vb_base / Vmax;
  else Mb = 0;
  if ((Vc_base != 0) && (Vmax != 0))  Mc = Vc_base / Vmax;
  else Mc = 0;
}
float sumErr = 0;

float kpA = 1, kpB = 1, kpC = 1;
float kiA = 0.001, kiB = 0.001, kiC = 0.001;
float kdA = 0.05, kdB = 0.05, kdC = 0.05;
float kpEncA = 1.2, kpEncB = 1.2, kpEncC = 1.2;
float kiEncA = 0.00001, kiEncB = 0.00001, kiEncC = 0.00001;
float kdEncA = 0.01, kdEncB = 0.01, kdEncC = 0.01;

int direction = 12;


void Rotation(int settingDegree)
{
  Va_base = 0;
  Vb_base = 0;
  Vc_base = 0;
  while (1)
  {

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    x = ((((int)euler.x() + 180) % 360) + correction) % 360;
    if ((x - settingDegree) < 0)
    {
      Va_base = -20;
      Vb_base = -20;
      Vc_base = -20;
    }
    else
    {
      Va_base = 20;
      Vb_base = 20;
      Vc_base = 20;
    }
    if (abs(x - settingDegree) <= 1)
      break;
    Va = (int)(Va_base);
    Vb = (int)(Vb_base);
    Vc = (int)(Vc_base);
    if (Va > 0)    roboclaw.ForwardM1(address1, abs(Va));
    else  roboclaw.BackwardM1(address1, abs(Va));
    if (Vb > 0)    roboclaw.ForwardM2(address2, abs(Vb));
    else  roboclaw.BackwardM2(address2, abs(Vb));
    if (Vc > 0)    roboclaw.ForwardM2(address1, abs(Vc));
    else  roboclaw.BackwardM2(address1, abs(Vc));
  }
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  roboclaw.SpeedM2(address1, 0);
  Va_base = 0;
  Vb_base = 0;
  Vc_base = 0;
  delay(10);
}



void pidForMotor()
{
  roboclaw.SpeedM1(address2, 0);
  delay(100);
  //  Serial.println(SettingXDegrees);
  velocity = 0;
  CalculateSpeed(0);
  CalculateKoef();
  resetAll();
  kpEncA = 0.4, kpEncB = 0.4, kpEncC = 0.4;
  kiEncA = 0, kiEncB = 0, kiEncC = 0;
  kdEncA = 0.3, kdEncB = 0.3, kdEncC = 0.3;
  float precent = 1;
  kpA = 2;
  kdA = 4;
  kiA = 0.0001;
  kpB = 2;
  kdB = 4;
  kiB = 0.0001;
  kpC = 2;
  kdC = 4;
  kiC = 0.0001;
  dt = 10;
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  roboclaw.SpeedM2(address1, 0);
  String data;
  uint8_t status1, status2, status3;
  uint8_t _status1, _status2, _status3;
  bool valid1, valid2, valid3;
  bool _valid1, _valid2, _valid3;
  int speed1, speed2, speed3;
  int i = 0;
  correction = 0;
  //Serial.println("Hello");
  String statusMes = "";
  String message = "";
  String stopMotors = "";
  String dir = "";
  String angleString = "";
  String _StringTick = "";
  int tick1 = 0;
  int tick2 = 0;
  int tick3 = 0;
  int _angle = 0;
  int smn = 0;
  velocity = 100;
  alpha = 90 * PI / 180;
  CalculateSpeed(0);
  CalculateKoef();
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
      //Serial.println(x);
      //Rotation(SettingXDegrees);
      rotationWithPID(_angle);
      resetAll();
      CalculateSpeed(0);
      CalculateKoef();
    }
    speed1 = roboclaw.ReadSpeedM1(address1, &_status1, &_valid1);
    speed2 = roboclaw.ReadSpeedM2(address2, &_status2, &_valid2);
    speed3 = roboclaw.ReadSpeedM2(address1, &_status3, &_valid3);
    speed1 = map(speed1, - 5000, 5000, -127, 127);
    speed2 = map(speed2, - 5000, 5000, -127, 127);
    speed3 = map(speed3, - 5000, 5000, -127, 127);
    enc1 = roboclaw.ReadEncM1(address1, &status1, &valid1);
    enc2 = roboclaw.ReadEncM2(address2, &status2, &valid2);
    enc3 = roboclaw.ReadEncM2(address1, &status3, &valid3);
    if ((abs(enc1) > 10000) || (abs(enc2) > 10000) || (abs(enc3) > 10000)) resetAll();
    if (valid1 && valid2 && valid3 && _valid1 && _valid2 && _valid3 )
    {
      Serial.print(String(speed1) + "\t" + String(speed2) + "\t" + String(speed3) + "\t");
      Serial.println(String(Va_base) + "\t" + String(Vb_base) + "\t" + String(Vc_base));
      switch (EncMaxNum)
      {
        case 1:
          maxEnc = enc1;
          break;
        case 2:
          maxEnc = enc2;
          break;
        case 3:
          maxEnc = enc3;
          break;
      }
      //maxEnc = min(enc1, min(enc2, enc3));
      errA = Va_base - speed1;
      errB = Vb_base - speed2;
      errC = Vc_base - speed3;
      Serial.println(String(errA) + "\t" + String(errB) + "\t" + String(errC));
      //maxEnc = min(enc1 / Ma, min(enc2 / Mb, enc3 / Mc));
      errEncA = Ma * maxEnc - enc1;
      errEncB = Mb * maxEnc - enc2;
      errEncC = Mc * maxEnc - enc3;
      Da = (errA - prevErrA) / dt;
      Db = (errB - prevErrB) / dt;
      Dc = (errC - prevErrC) / dt;
      DEncA = (errEncA - prevErrEncA) / dt;
      DEncB = (errEncB - prevErrEncB) / dt;
      DEncC = (errEncC - prevErrEncC) / dt;
      prevErrA = errA;
      prevErrB = errB;
      prevErrC = errC;
      prevErrEncA = errEncA;
      prevErrEncB = errEncB;
      prevErrEncC = errEncC;
      integralA = integralA + errA * dt;
      integralB = integralB + errB * dt;
      integralC = integralC + errC * dt;
      Ua = kpA * errA + Da * kdA + integralA * kiA;
      Ub = kpB * errB + Db * kdB + integralB * kiB;
      Uc = kpC * errC + Dc * kdC + integralC * kiC;
      UEncA = (kpEncA * errEncA + DEncA * kdEncA + integralEncA * kiEncA);
      UEncB = (kpEncB * errEncB + DEncB * kdEncB + integralEncB * kiEncB);
      UEncC = (kpEncC * errEncC + DEncC * kdEncC + integralEncC * kiEncC);
      Va = (int)(Va_base + UEncA * precent + Ua * (float)(1.0 - precent));
      Vb = (int)(Vb_base + UEncB * precent + Ub * (float)(1.0 - precent));
      Vc = (int)(Vc_base + UEncC * precent + Uc * (float)(1.0 - precent));
      if (Va > 0)    roboclaw.ForwardM1(address1, abs(Va));
      else  roboclaw.BackwardM1(address1, abs(Va));
      if (Vb > 0)    roboclaw.ForwardM2(address2, abs(Vb));
      else  roboclaw.BackwardM2(address2, abs(Vb));
      if (Vc > 0)    roboclaw.ForwardM2(address1, abs(Vc));
      else  roboclaw.BackwardM2(address1, abs(Vc));
      //Serial.println(String(enc1) + "\t" + String(enc2) + "\t" + String(enc3));
    }
    else
    {
      //resetAll();
      Serial.println(String(enc1) + "\t" + String(enc2) + "\t" + String(enc3));
      Serial.println(String(speed1) + "\t" + String(speed2) + "\t" + String(speed3));
      Serial.println("error");
    }
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
