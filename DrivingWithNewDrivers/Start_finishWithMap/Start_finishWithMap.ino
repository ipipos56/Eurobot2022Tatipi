#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <Wire.h>
#include <Servo.h>

SoftwareSerial serial(10, 11);
SoftwareSerial serial(12, 13);
RoboClaw roboclaw(&serial, 10000);
RoboClaw roboclaw2(&seria2, 10000);
DynamicJsonDocument doc(2048);



Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);


#define address1 0x80
#define address2 0x81

#define pi PI

#define R 0.058/2 * 100
#define _R 0.16 * 100
#define Radius 0.32 * 100
#define Kp 6
#define Ki 0
#define Kd 7.5
#define qpps 3000

uint32_t myTimer1;
int period = 100;
float angleA, angleB, angleC;
long int enc1;
long int enc2;
long int enc3;
float x = 0;
int xx = 0;
int correction = 0;
bool _finishMoving = 0;

float alpha = 0;
float Va = 0;
float Vb = 0;
float Vc = 0;
int Va_base = 0;
int Vb_base = 0;
int Vc_base = 0;
int velocity;
int target = 2350 * 1;
float sta_fin;
bool OnTarget = false;
int EndTimer = millis();
int error = 0;
extern volatile unsigned long timer0_millis;

int analogPin = 0;
int raw = 0;
int Vin = 5;
float Vout = 0;
float R1 = 1000;
float R2 = 0;
float buffer = 0;

#define BNO055_SAMPLERATE_DELAY_MS (period)
void CalculateSpeed(int _angle)
{

  //  angleA = cos((150 * PI / 180 - _angle * PI / 180 - alpha));
  //  angleB = cos((30 * PI / 180 - _angle * PI / 180 - alpha));
  //  angleC = cos((270 * PI / 180 - _angle * PI / 180 - alpha));
  angleA = cos((150 * PI / 180 - _angle * PI / 180 - alpha - 60 * PI / 180));
  angleB = cos((30 * PI / 180 - _angle * PI / 180 - alpha - 60 * PI / 180));
  angleC = cos((270 * PI / 180 - _angle * PI / 180 - alpha - 60 * PI / 180));
  Va_base = velocity * angleA;
  Vb_base = velocity * angleB;
  Vc_base = velocity * angleC;
}
void(* resetFunc) (void) = 0;
void pidForMotor()
{
  roboclaw.SetM1VelocityPID(address1, Kd, Kp, Ki, qpps);
  roboclaw.SetM2VelocityPID(address2, Kd, Kp, Ki, qpps);
  roboclaw.SetM2VelocityPID(address1, Kd, Kp, Ki, qpps);
  roboclaw.ForwardM1(address1, 0);
  roboclaw.ForwardM2(address2, 0);
  roboclaw.ForwardM2(address1, 0);
  int smn = 0;
  int _angle = 0;
  int prevAngle = 0;
  String data;
  String statusString = "";
  String angleString = "";
  String speedString = "";
  String targetString = "";
  velocity = 0;
  alpha = 0 * PI / 180;
  CalculateSpeed(_angle);
  delay(50);
  roboclaw.SetEncM1(address1, 0);
  roboclaw.SetEncM2(address2, 0);
  roboclaw.SetEncM2(address1, 0);
  while (1)
  {
    if ((millis() - myTimer1) >= period) {   // ищем разницу (500 мс)
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
    if (Serial.available())
    {
      data = Serial.readStringUntil('\n');
      char* json = data.c_str();
      deserializeJson(doc, json);
      statusString = doc[String("s")].as<String>();
      if (statusString == "1")
      {
        speedString = doc[String("sp")].as<String>();
        angleString = doc[String("a")].as<String>();
        targetString = doc[String("t")].as<String>();
        target = targetString.toInt();
        velocity = speedString.toInt();
        if (velocity > 0)
        {
          roboclaw.SetEncM1(address1, 0);
          roboclaw.SetEncM2(address2, 0);
          roboclaw.SetEncM2(address1, 0);
        }
        else
        {
          roboclaw.SpeedM1(address1, 0);
          roboclaw.SpeedM2(address2, 0);
          roboclaw.SpeedM2(address1, 0);
        }
        alpha = angleString.toInt();
        alpha = alpha * PI / 180;
        CalculateSpeed(x);
        //{"s":"1","sp":"3000","t":"1000","a":"45"}
        //{"s":"2","a":"-90"}
      }
      if (statusString == "2")
      {
        angleString = doc[String("a")].as<String>();
        _angle = angleString.toInt();
        prevAngle = _angle;
        _angle = normilize(_angle);
        smn = _angle - x;
        if (smn > 180)
          smn -= 360;
        else if (smn < -180)
          smn += 360;
        rotationWithPID(_angle);
        resetFunc();
      }
      if (statusString == "3")
      {
        bool rawBool = 0;
        int _timer3 = millis();
        while (!rawBool)
        {
          int timer3 = millis();
          raw = analogRead(analogPin);
          if (raw)
          {
            buffer = raw * Vin;
            Vout = (buffer) / 1024.0;
            buffer = (Vin / Vout) - 1;
            R2 = R1 * buffer;
            //Serial.print("R2: ");
            //Serial.println(R2);
            rawBool = !rawBool;
          }
          if ((timer3 - _timer3) > 500)
            break;
          }
          if(rawBool)
          {
            Serial.println("R:" + String(R2));
          }
          else
          {
            Serial.println("Again");
          }
      }
    }
    enc1 = roboclaw.ReadEncM1(address1);
    enc2 = roboclaw.ReadEncM2(address2);
    enc3 = roboclaw.ReadEncM2(address1);
    float v = (float)(0.6667) * ((float)enc1 * angleA + (float)enc2 * angleB + (float)enc3 * angleC);
    if ((v >= target) && (!OnTarget)) {
      EndTimer = millis();
      OnTarget = true;
    }
    CalculateSpeed(x);
    //Serial.println(String(Va_base) + "\t" + String(Vb_base) + "\t" + String(Vc_base));
    if (OnTarget && (millis() - EndTimer > 300))
    {
      OnTarget = false;
      velocity = 0;
      roboclaw.SetEncM1(address1, 0);
      roboclaw.SetEncM2(address2, 0);
      roboclaw.SetEncM2(address1, 0);
      CalculateSpeed(x);
      Serial.println("v :\t" + String(enc1 + enc2 + enc3) + "\t" + String(enc1) + "\t" + String(enc2) + "\t" + String(enc3));
      rotationWithPID(prevAngle);
      v = 0;
      resetFunc();
    }

    sta_fin = min(1.0, (float)((float)target - v) / (target * 0.15)) * min(1.0, (float)(v + 10.0) / (target * 0.15));
    roboclaw.SpeedM1(address1, (int)(Va_base * sta_fin));
    roboclaw.SpeedM2(address2, (int)(Vb_base * sta_fin));
    roboclaw.SpeedM2(address1, (int)(Vc_base * sta_fin));
  }
}

void setup()
{
  Serial.begin(38400);
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
    {
      Serial.println("Reconnect");
    }
  }
  Serial.println("Start");
  bno.setExtCrystalUse(true);
  timer0_millis = UINT32_MAX - 5000;
  roboclaw.begin(38400);
  roboclaw2.begin(38400);
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
  float kDifRotation = 9, kIntRotation = 0.5, kPrRotation = 8;
  float _dt = 10;
  int controlAct = 0;
  int smn = 0;
  roboclaw.ForwardM1(address1, 0);
  roboclaw.ForwardM2(address2, 0);
  roboclaw.ForwardM2(address1, 0);
  delay(50);
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
      // Serial.println(x);
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
    int speedA = roboclaw.ReadSpeedM1(address1);
    int speedB = roboclaw.ReadSpeedM2(address2);
    int speedC = roboclaw.ReadSpeedM2(address1);
    if ((abs(speedA) == 0) && (abs(speedB) == 0) && (abs(speedC) == 0) && (abs(rotationErr) < 1.0))
    {
      roboclaw.ForwardM1(address1, 0);
      roboclaw.ForwardM2(address2, 0);
      roboclaw.ForwardM2(address1, 0);
      break;
    }
    //    if (abs(rotationErr) < 1)
    //    {
    //      roboclaw.ForwardM1(address1, 0);
    //      roboclaw.ForwardM2(address2, 0);
    //      roboclaw.ForwardM2(address1, 0);
    //      break;
    //    }    delay(1);
  }
  roboclaw.ForwardM1(address1, 0);
  roboclaw.ForwardM2(address2, 0);
  roboclaw.ForwardM2(address1, 0);
}

void loop()
{
  pidForMotor();
}
