#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <NewPing.h>
#include <Wire.h>
#include <Servo.h>
#include "RoboClaw.h"

#define velocity 3000

#define HC_TRIG 6
#define HC_ECHO 7
#define MAX_DISTANCE 20
#define SPEED_OF_ROTATION 1000

#define address1 0x80
#define address2 0x81
#define address3 0x82

#define Kp1 4
#define Ki1 0.5
#define Kd1 7
#define qpps1 4000

#define Kp2 4
#define Ki2 0.5
#define Kd2 7
#define qpps2 4000

#define Kp3 4
#define Ki3 0.5
#define Kd3 7
#define qpps3 4000

#define Kp4 4
#define Ki4 0.5
#define Kd4 7
#define qpps4 4000

#define Kp5 4
#define Ki5 0.5
#define Kd5 7
#define qpps5 4000

#define Kp6 4
#define Ki6 0.5
#define Kd6 7
#define qpps6 4000

#define BNO055_SAMPLERATE_DELAY_MS (50)

extern volatile unsigned long timer0_millis;

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);

Servo arm11;
Servo arm12;
Servo arm21;
Servo arm22;

SoftwareSerial serial1(10, 11);

RoboClaw roboclaw1(&serial1, 50000);

NewPing sonar(HC_TRIG, HC_ECHO, MAX_DISTANCE);

DynamicJsonDocument doc(2048);

float lengthOfRobot = 0;
float alpha = 0;
float angleA, angleB, angleC;
float Va_base = 0, Vb_base = 0, Vc_base = 0;
int staticAngle = 180;
int previousAngle = 0;
int previousTick = 0;
long int enc1 = 0, enc2 = 0, enc3 = 0, enc4 = 0;
String data = "", externalData = "";

void(* resetFunc) (void) = 0;
void CalculateSpeed(int _angle);
void cameraRotation(int _angle, int stat);
void uart();
void decoderOfOperation(int status);
void firstStart();
void Sender(bool reset);

int move(int angle, long int length);
int decoder(String _data);
int normilize(int angle);
int rotationWithPID(int _angle);

void setup()
{
  timer0_millis = UINT32_MAX - 5000;
  Serial.begin(115200);
  Serial.flush();
  roboclaw1.begin(115200);
  if (!bno.begin())
  {
    while (1)
    {
      delay(50);
      resetFunc();
    }
  }
  roboclaw1.SetM1VelocityPID(address1, Kd1, Kp1, Ki1, qpps1);
  roboclaw1.SetM1VelocityPID(address2, Kd2, Kp2, Ki2, qpps2);
  roboclaw1.SetM1VelocityPID(address3, Kd3, Kp3, Ki3, qpps3);
  roboclaw1.SetM2VelocityPID(address1, Kd4, Kp4, Ki4, qpps4);
  roboclaw1.SetEncM2(address1, 0);
  Serial.flush();
  //bno.setExtCrystalUse(true);
}
void loop()
{
  uart();
  //roboclaw1.ForwardM2(address2, 0);
  //Sender(0);
  //delay(500);
}

void cameraRotation(int _angle)
{
  float cRotationErr = 0, cPreviousRotationErr = 0;
  float cDifRotation = 0, cIntRotation = 0, cPrRotation = 0;
  float cKDifRotation = 8, cKIntRotation = 0.5, cKPrRotation = 7;
  float _dt = 10;
  int controlAct = 0;
  int angle = _angle - previousAngle;
  previousAngle = _angle;
  int trueTick = (int)(800 * (float)(60.0 / 20.0)) * (float)(angle / 180.0);
  int tick;
  while (1)
  {
    tick = roboclaw1.ReadEncM2(address1);
    cRotationErr = trueTick - tick + previousTick;
    cPrRotation = cRotationErr * cKPrRotation;
    cDifRotation = (float)((cRotationErr - cPreviousRotationErr) / _dt) * cKDifRotation;
    cPreviousRotationErr = cRotationErr;
    if (cRotationErr > 0)
      controlAct = (int)(cPrRotation + cDifRotation + cIntRotation);
    else
      controlAct = (int)(cPrRotation + cDifRotation + cIntRotation);
    roboclaw1.SpeedM2(address1, controlAct);
    int speedD = roboclaw1.ReadSpeedM2(address1);
    if ((abs(speedD) == 0) && (abs(cRotationErr) < 1.0))
    {
      roboclaw1.SpeedM2(address1, 0);
      break;
    }
  }
  previousTick = tick;
}

void Sender(bool reset)
{
  String send = "";
  switch (reset)
  {
    case 0:
      send += '{';
      send += '"';
      send += 'r';
      send += '"';
      send += ':';
      send += '"';
      send += '0';
      send += '"';
      send += ',';
      send += '"';
      send += 'a';
      send += '"';
      send += ':';
      send += '"';
      break;
    case 1:
      send += '{';
      send += '"';
      send += 'r';
      send += '"';
      send += ':';
      send += '"';
      send += '1';
      send += '"';
      send += ',';
      send += '"';
      send += 'a';
      send += '"';
      send += ':';
      send += '"';
      break;
  }
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int x = euler.x();
  x = 360 - x;
  send += String(x, DEC);
  send += '"';
  send += '}';
  Serial.println(send);
  delay(50);
}

void firstStart()
{
  roboclaw1.SpeedM2(address1, 0);
  roboclaw1.SetEncM2(address1, 0);
  int cm;
  bool logic = 0;
  while (logic == 0)
  {
    roboclaw1.SpeedM2(address1, -2000);
    cm = sonar.ping_cm();
    if (cm > 4 && cm < 10)
    {
      roboclaw1.SpeedM2(address1, 0);
      delay(10);
      roboclaw1.SetEncM2(address1, 0);
      while (1)
      {
        roboclaw1.SpeedM2(address1, -2000);
        enc4 = roboclaw1.ReadEncM2(address1);
        if (abs(enc4) > 150)
        {
          roboclaw1.SpeedM2(address1, 0);
          logic = 1;
          break;
        }
      }
    }
  }
}

void CalculateSpeed(int _angle)
{
  angleA = cos((150 * PI / 180 + _angle * PI / 180 - alpha * PI / 180 - staticAngle * PI / 180));
  angleB = cos((30 * PI / 180 + _angle * PI / 180 - alpha * PI / 180 - staticAngle * PI / 180));
  angleC = cos((270 * PI / 180 + _angle * PI / 180 - alpha * PI / 180 - staticAngle * PI / 180));
  Va_base = velocity * angleA;
  Vb_base = velocity * angleB;
  Vc_base = velocity * angleC;
}

void uart()
{
  if (Serial.available() > 0)
  {
    data = Serial.readStringUntil('\n');
    char* json = data.c_str();
    DeserializationError error =  deserializeJson(doc, json);
    if (!error)
    {
      String statusString = doc[String("s")].as<String>();
      String aString = doc[String("a")].as<String>();
      String bString = doc[String("b")].as<String>();
      int status = statusString.toInt();
      long int a = aString.toInt();
      long int b = bString.toInt();
      if (status == 0)
      {
        Sender(1);
        resetFunc();
      }
      else if (status == 1)
      {
        int movingStaus = move(a, b);
        delay(50);
        Sender(0);
      }
      else if (status == 2)
      {
        cameraRotation(a);
        delay(50);
        Sender(0);
      }
      else if (status == 3)
      {
        rotationWithPID(a);
        Sender(0);
      }
      else if (status == 4)
      {
        arm11.attach(53);
        arm12.attach(52);
        b = 180 - b;
        arm11.write(a);
        arm12.write(b);
        delay(1000);
        arm11.detach();
        arm12.detach();
        Sender(0);
      }
      else if (status == 5)
      {
        arm21.attach(51);
        arm22.attach(50);
        b = 180 - b;
        arm21.write(a);
        arm22.write(b);
        delay(1000);
        arm21.detach();
        arm22.detach();
        Sender(0);
      }
      else if (status == 6)
      {
        if (a == 1)
        {
          if (b == 1)
          {
            roboclaw1.ForwardM2(address2, 100);
          }
          else if (b == 0)
          {
            roboclaw1.ForwardM2(address2, 0);
          }
        }
        else if (a == 2)
        {
          if (b == 1)
          {
            roboclaw1.ForwardM2(address3, 60);
          }
          else if (b == 0)
          {
            roboclaw1.ForwardM2(address3, 0);
          }
        }
        Sender(0);
      }
    }
  }
}

// 15000 ~ 1140, 30000 ~ 2275
int rotationWithPID(int _angle)
{
  int startRot = millis();
  float rotationErr = 0, previousRotationErr = 0;
  float difRotation = 0, intRotation = 0, prRotation = 0;
  float kDifRotation = 20, kIntRotation = 0.05, kPrRotation = 50;
  float _dt = 5;
  int controlAct = 0;
  int logic = 0;
  roboclaw1.SetEncM1(address1, 0);
  roboclaw1.SetEncM1(address2, 0);
  roboclaw1.SetEncM1(address3, 0);
  while (logic == 0)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    x = 360 - x;
    _angle = normilize(_angle);
    int smn = _angle - x;
    if (smn > 180)
      smn -= 360;
    else if (smn < -180)
      smn += 360;
    alpha = smn;
    CalculateSpeed(x);
    rotationErr = smn;
    prRotation = rotationErr * kPrRotation;
    difRotation = ((rotationErr - previousRotationErr) / _dt) * kDifRotation;
    previousRotationErr = rotationErr;
    intRotation = (intRotation + rotationErr * _dt) * kIntRotation;
    if (rotationErr > 0)
      controlAct = (int)(prRotation + difRotation + intRotation);
    else
      controlAct = (int)(prRotation + difRotation + intRotation);
    roboclaw1.SpeedM1(address1, controlAct);
    roboclaw1.SpeedM1(address2, controlAct);
    roboclaw1.SpeedM1(address3, controlAct);
    int speedA = roboclaw1.ReadSpeedM1(address1);
    int speedB = roboclaw1.ReadSpeedM1(address2);
    int speedC = roboclaw1.ReadSpeedM1(address3);
    if ((abs(speedA) == 0) && (abs(speedB) == 0) && (abs(speedC) == 0) && (abs(rotationErr) < 1.0))
    {
      roboclaw1.SpeedM1(address1, 0);
      roboclaw1.SpeedM1(address2, 0);
      roboclaw1.SpeedM1(address3, 0);
      logic = 1;
    }
  }
  Serial.println(String(millis() - startRot));
  return logic;
}

int normilize(int angle)
{
  angle = angle % 360;
  if (angle < 0)
    angle += 360;
  return angle;
}

//int decoder(String _data)
//{
//  char* json = _data.c_str();
//  DeserializationError error = deserializeJson(doc, json);
//  delay(10);
//  if (error)
//  {
//    return -1;
//  }
//  else
//  {
//    return doc[String("s")].as<String>().toInt();
//  }
//}

int move(int angle, long int length)
{
  int logic = 0;
  roboclaw1.SetEncM1(address1, 0);
  roboclaw1.SetEncM1(address2, 0);
  roboclaw1.SetEncM1(address3, 0);
  lengthOfRobot = 0;
  externalData = "";
  delay(100);
  alpha = angle;
  while (logic == 0)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    x = 360 - x;
    CalculateSpeed(x);
    enc1 = roboclaw1.ReadEncM1(address1);
    enc2 = roboclaw1.ReadEncM1(address2);
    enc3 = roboclaw1.ReadEncM1(address3);
    roboclaw1.SpeedM1(address1, (int)(Va_base));
    roboclaw1.SpeedM1(address2, (int)(Vb_base));
    roboclaw1.SpeedM1(address3, (int)(Vc_base));
    lengthOfRobot = (float)(0.6667) * (enc1 * angleA + enc2 * angleB + enc3 * angleC);
    if (lengthOfRobot >= length)
    {
      externalData = "";
      logic = 1;
    }
    delay(1);
  }
  roboclaw1.SpeedM1(address1, 0);
  roboclaw1.SpeedM1(address2, 0);
  roboclaw1.SpeedM1(address3, 0);
  return logic;
}
