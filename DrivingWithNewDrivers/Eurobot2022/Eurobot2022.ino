#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <Servo.h>
#include "RoboClaw.h"

#define velocity 3000

#define address1 0x80
#define address2 0x81
#define address3 0x81

#define Kp1 4
#define Ki1 0.15
#define Kd1 0
#define qpps1 4000

#define Kp2 4
#define Ki2 0.15
#define Kd2 0
#define qpps2 4000

#define Kp3 4
#define Ki3 0.15
#define Kd3 0
#define qpps3 4000

#define BNO055_SAMPLERATE_DELAY_MS (50)

#define arm1 65
#define arm2 95
#define maniX 0
#define maniY 160

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);

extern volatile unsigned long timer0_millis;

Servo arm11;
Servo arm12;
Servo kut;

SoftwareSerial serial1(10, 11);

RoboClaw roboclaw1(&serial1, 50000);

DynamicJsonDocument doc(2048);

int analogPin = 1;
int raw = 0;
int Vin = 5;
float Vout = 0;
float R1 = 1000;
float R2 = 0;
float buffer = 0;

float lengthOfRobot = 0;
float alpha = 0;
float angleA, angleB, angleC;
float Va_base = 0, Vb_base = 0, Vc_base = 0;
float alphat = 0, beta = 0;
int staticAngle = 0;
int movingStatus = 0;
long int enc1 = 0, enc2 = 0, enc3 = 0, enc4 = 0;
int cX = 0, cY = 0, AC = 0;
String data = "", externalData = "";

void maniKinematics(int x, int y);
void(* resetFunc) (void) = 0;
void CalculateSpeed(int _angle);
void uart();
void Sender(int reset, int _tick);

int move(int angle, long int length);
int normilize(int angle);
int rotationWithPID(int _angle);

void setup()
{
  pinMode(A0, INPUT);
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
  roboclaw1.SetM2VelocityPID(address2, Kd2, Kp2, Ki2, qpps2);
  roboclaw1.SetM1VelocityPID(address3, Kd3, Kp3, Ki3, qpps3);
  roboclaw1.SetEncM1(address1, 0);
  roboclaw1.SetEncM2(address2, 0);
  roboclaw1.SetEncM1(address3, 0);
  Serial.flush();
  int kno =  digitalRead(A0);
  if (kno == 0)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    x = 360 - x;
    Sender(2, x, 0);
  }
  else
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    x = 360 - x;
    Sender(3, x, 0);
  }
}

void loop() {
  uart();

}

void maniKinematics(int _x, int _y)
{
  /*
    cX = _x;
    cY = _y;
    AC = (int)sqrt((cX - maniX) * (cX - maniX) + (cY - maniY) * (cY - maniY));
    alphat = acos((float)((float)(AC * AC + arm1 * arm1 - arm2 * arm2) / (float)(2 * AC * arm1)));
    beta = acos((float)((float)(arm1 * arm1 + arm2 * arm2 - AC * AC) / (float)(2 * arm1 * arm2)));
    float angleBt = PI - beta;
    float _angleA = PI - atan2((float)(cX - maniX) , (float)(cY - maniY));
    float angleAt =  _angleA - alphat;
    int servo1Angle;
    int servo2Angle;

    if (angleA < 0)
    {
    angleAt = angleAt + 2 * alphat;
    int _angleB = (angleBt * 180.0) / PI;
    _angleB = 90 - constrain(_angleB, 0 , 90);
    angleBt = PI / 2 - angleBt;
    servo1Angle = 180 - ((int)(((float)angleAt / PI) * 180));
    servo2Angle = (int)(((float)angleBt / PI) * 180);
    }
    else
    {
    servo1Angle = 180 - ((int)(((float)angleAt / PI) * 180));
    servo2Angle = (int)(((float)angleBt / PI) * 180) + 90;
    }
  */
  int servo1Angle;
  int servo2Angle;
  servo1Angle = 180 - _x;
  servo2Angle = _y;
  arm11.attach(52);
  arm12.attach(53);

  if ((!isnan(servo1Angle)) || (!isnan(servo1Angle)))
  {
    arm11.write(servo1Angle);
    arm12.write(servo2Angle);
  }
  else
  {
    arm11.write(180);
    arm12.write(95);
  }
  delay(100);
  arm11.detach();
  arm12.detach();
}


void Sender(int reset, int _an, int _tick)
{
  String send = "";
  send += '{';
  send += '"';
  send += 'r';
  send += '"';
  send += ':';
  send += '"';
  send += String(reset, DEC);
  send += '"';
  send += ',';
  send += '"';
  send += 'a';
  send += '"';
  send += ':';
  send += '"';
  send += String(_an, DEC);
  send += '"';
  send += ',';
  send += '"';
  send += 't';
  send += '"';
  send += ':';
  send += '"';
  send += String(_tick, DEC);
  send += '"';
  send += '}';
  Serial.println(send);
  delay(50);
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
  if ((Serial.available() > 0) || (movingStatus == 1))
  {
    if (movingStatus == 1)
    {
      data = externalData;
      movingStatus = 0;
    }
    else
    {
      data = Serial.readStringUntil('\n');
    }
    char* json = data.c_str();
    DeserializationError error =  deserializeJson(doc, json);
    if (!error)
    {
      String statusString = doc[String("s")].as<String>();
      String aString = doc[String("a")].as<String>();
      String bString = doc[String("b")].as<String>();
      int status = statusString.toInt();
      int a = aString.toInt();
      int b = bString.toInt();
      delay(100);
      //Serial.println(String(status) + "\t" + String(aString) + "\t" + String(bString));
      Serial.flush();
      if (status == 0)
      {
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        int x = euler.x();
        x = 360 - x;
        Sender(1, x, 0);
        resetFunc();
      }
      else if (status == 1)
      {
        move(a, b);
        delay(50);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        int x = euler.x();
        x = 360 - x;
        Sender(0, x, lengthOfRobot);
      }
      else if (status == 2)
      {
        int rawBool = 0;
        raw = analogRead(analogPin);
        if (raw)
        {
          buffer = raw * Vin;
          Vout = (buffer) / 1024.0;
          buffer = (Vin / Vout) - 1;
          R2 = R1 * buffer;
          if (R2 > 300 && R2 < 500)
            rawBool = 3;
          else if (R2 > 800 && R2 < 1200)
            rawBool = 2;
          else if (R2 > 4500 && R2 < 5000)
            rawBool = 1;
        }
        else
        {
          rawBool = 0;
        }
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        int x = euler.x();
        x = 360 - x;
        Sender(4, x, rawBool);
      }
      else if (status == 3)
      {
        rotationWithPID(a);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        int x = euler.x();
        x = 360 - x;
        Sender(0, x, 0);
      }
      else if (status == 4)
      {
        maniKinematics(a, b);
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        int x = euler.x();
        x = 360 - x;
        Sender(0, x, 0);
      }
      else if (status == 5)
      {
        if (a == 1)
        {
          roboclaw1.ForwardM2(address1, 60);
        }
        else if (a == 0)
        {
          roboclaw1.ForwardM2(address1, 0);
        }
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        int x = euler.x();
        x = 360 - x;
        Sender(0, x, 0);
      }
      else if (status == 6)
      {
        kut.attach(50);
        kut.write(a);
        delay(1000);
        kut.detach();
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        int x = euler.x();
        x = 360 - x;
        Sender(0, x, 0);

      }
    }
  }
}

int rotationWithPID(int _angle)
{
  uint32_t myTimer1;
  int period = 50;
  int startRot = millis();
  int rotationErr = 0, previousRotationErr = 0;
  float difRotation = 0, intRotation = 0, prRotation = 0;
  float kDifRotation = 10, kIntRotation = 0.5, kPrRotation = 20;
  float _dt = 0.05;
  int controlAct = 0;
  int logic = 0;
  roboclaw1.SetEncM1(address1, 0);
  roboclaw1.SetEncM2(address2, 0);
  roboclaw1.SetEncM1(address3, 0);
  while (logic == 0)
  {
    if ((millis() - myTimer1) >= period) {   // ищем разницу (500 мс)
      myTimer1 += period;
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
      difRotation = ((rotationErr - previousRotationErr)) * kDifRotation;
      previousRotationErr = rotationErr;
      intRotation = (intRotation + rotationErr * _dt) * kIntRotation;
      if (rotationErr > 0)
        controlAct = (int)(prRotation + difRotation + intRotation);
      else
        controlAct = (int)(prRotation + difRotation + intRotation);
    }
    roboclaw1.SpeedM1(address1, controlAct);
    roboclaw1.SpeedM2(address2, controlAct);
    roboclaw1.SpeedM1(address3, controlAct);
    int speedA = roboclaw1.ReadSpeedM1(address1);
    int speedB = roboclaw1.ReadSpeedM2(address2);
    int speedC = roboclaw1.ReadSpeedM1(address3);
    if ((abs(speedA) < 4) && (abs(speedB) < 4) && (abs(speedC) < 4) && (abs(rotationErr) < 1.0))
    {
      roboclaw1.SpeedM1(address1, 0);
      roboclaw1.SpeedM2(address2, 0);
      roboclaw1.SpeedM1(address3, 0);
      logic = 1;
    }
  }
  //Serial.println(String(millis() - startRot));
  return logic;
}

int normilize(int angle)
{
  angle = angle % 360;
  if (angle < 0)
    angle += 360;
  return angle;
}

int move(int angle, long int length)
{
  int logic = 0;
  int sta_fin = 0;
  roboclaw1.SetEncM1(address1, 0);
  roboclaw1.SetEncM2(address2, 0);
  roboclaw1.SetEncM1(address3, 0);
  lengthOfRobot = 0;
  externalData = "";
  delay(100);
  alpha = angle;
  int _extra = 0;
  while (logic == 0)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    x = 360 - x;
    CalculateSpeed(x);
    enc1 = roboclaw1.ReadEncM1(address1);
    enc2 = roboclaw1.ReadEncM2(address2);
    enc3 = roboclaw1.ReadEncM1(address3);
    lengthOfRobot = (float)(0.6667) * (enc1 * angleA + enc2 * angleB + enc3 * angleC);
    float fin = min(1.0, (float)(lengthOfRobot + 200) / (float)(1000));
    float st = min(1.0, (float)(length - lengthOfRobot) / (float)(400));
    //Serial.println(String(fin) + "\t" + String(st));
    roboclaw1.SpeedM1(address1, (int)(Va_base * fin * st));
    roboclaw1.SpeedM2(address2, (int)(Vb_base * fin * st));
    roboclaw1.SpeedM1(address3, (int)(Vc_base * fin * st));
    int speedA = roboclaw1.ReadSpeedM1(address1);
    int speedB = roboclaw1.ReadSpeedM2(address2);
    int speedC = roboclaw1.ReadSpeedM1(address3);
    if ((abs(speedA) < 1) && (abs(speedB) < 1) && (abs(speedC) < 1) && (length - lengthOfRobot) < 10)
    {
      roboclaw1.SpeedM1(address1, 0);
      roboclaw1.SpeedM2(address2, 0);
      roboclaw1.SpeedM1(address3, 0);
      logic = 1;
    }
    if (Serial.available() > 0)
    {
      data = Serial.readStringUntil('\n');
      String buf = String(data);
      char* json = data.c_str();
      DeserializationError error =  deserializeJson(doc, json);
      if (!error)
      {
        String statusString = doc[String("s")].as<String>();
        String aString = doc[String("a")].as<String>();
        String bString = doc[String("b")].as<String>();
        if (statusString.toInt() == 1)
        {
          externalData = String(buf);
          _extra = 1;
          movingStatus = _extra;
        }
        length = lengthOfRobot + 400;
      }
    }
    if ((lengthOfRobot >= length))
    {
      if (_extra == 0)
        externalData = "";
      logic = 1;
    }
  }
  roboclaw1.SpeedM1(address1, 0);
  roboclaw1.SpeedM2(address2, 0);
  roboclaw1.SpeedM1(address3, 0);
  return lengthOfRobot;
}
