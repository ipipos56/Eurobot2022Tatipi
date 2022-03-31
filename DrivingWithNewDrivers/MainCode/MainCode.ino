#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <Wire.h>
#include <Servo.h>

SoftwareSerial serial1(10, 11);

RoboClaw roboclaw1(&serial1, 50000);

DynamicJsonDocument doc(2048);

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);

#define velocity 3000

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

#define BNO055_SAMPLERATE_DELAY_MS (50)

String externalData = "";
int staticAngle = 0;
float lengthOfRobot = 0;
float alpha = 0;
float angleA, angleB, angleC;
float Va_base = 0, Vb_base = 0, Vc_base = 0;
long int enc1 = 0, enc2 = 0, enc3 = 0;
extern volatile unsigned long timer0_millis;

void(* resetFunc) (void) = 0;
void CalculateSpeed(int _angle);
int decoderOfOperation(int status);
int move(int angle, long int length);
int decoder(String _data);
int normilize(int angle);
int rotationWithPID(int _angle);
String uart();

void setup() 
{
  timer0_millis = UINT32_MAX - 5000;
  Serial.begin(115200);
  roboclaw1.begin(115200);
  if (!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1)
    {
      Serial.println("Reconnect");
      delay(50);
      resetFunc();
    }
  }
  //bno.setExtCrystalUse(true);
}
void loop() 
{
  decoderOfOperation(decoder(uart()));
}
void CalculateSpeed(int _angle)
{
  angleA = cos((150 * PI / 180 - _angle * PI / 180 - alpha * PI / 180 - staticAngle * PI / 180));
  angleB = cos((30 * PI / 180 - _angle * PI / 180 - alpha * PI / 180 - staticAngle * PI / 180));
  angleC = cos((270 * PI / 180 - _angle * PI / 180 - alpha * PI / 180 - staticAngle * PI / 180));
  Va_base = velocity * angleA;
  Vb_base = velocity * angleB;
  Vc_base = velocity * angleC;
}
String uart()
{
  String data = "";
  if (Serial.available())
    data = Serial.readStringUntil('\n');
  return data;
}

int rotationCam(int _angle)
{
  
}

int rotationWithPID(int _angle)
{
  float rotationErr = 0, previousRotationErr = 0;
  float difRotation = 0, intRotation = 0, prRotation = 0;
  float kDifRotation = 9, kIntRotation = 0.5, kPrRotation = 8;
  float _dt = 10;
  int controlAct = 0;
  int logic = 0;
  roboclaw1.SetEncM1(address1, 0);
  roboclaw1.SetEncM1(address2, 0);
  roboclaw1.SetEncM1(address3, 0);
  while (logic == 0)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    _angle = normilize(_angle);
    int smn = _angle - x;
    if (smn > 180)
      smn -= 360;
    else if (smn < -180)
      smn += 360;
    rotationErr = smn;
    prRotation = rotationErr * kPrRotation;
    difRotation = ((rotationErr - previousRotationErr) / _dt) * kDifRotation;
    previousRotationErr = rotationErr;
    intRotation = (intRotation + rotationErr * _dt) * kIntRotation;
    if (rotationErr > 0)
      controlAct = (int)(prRotation + difRotation + intRotation);
    else
      controlAct = (int)(prRotation + difRotation + intRotation);
    controlAct = constrain(abs(controlAct), 0, 127);
    if (rotationErr > 0)
    {
      roboclaw1.BackwardM1(address1, abs(controlAct));
      roboclaw1.BackwardM1(address2, abs(controlAct));
      roboclaw1.BackwardM1(address1, abs(controlAct));
    }
    else
    {
      roboclaw1.ForwardM1(address1, abs(controlAct));
      roboclaw1.ForwardM1(address2, abs(controlAct));
      roboclaw1.ForwardM1(address3, abs(controlAct));
    }
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
  return logic;
}

int normilize(int angle)
{
  angle = angle % 360;
  if (angle < 0)
    angle += 360;
  return angle;
}

int decoder(String _data)
{
  char* json = _data.c_str();
  DeserializationError error = deserializeJson(doc, json);
  delay(10);
  if (error)
  {
    return -1;
  }
  else
  {
    return doc[String("s")].as<String>().toInt();
  }
}

int move(int angle, long int length)
{
  int logic = 0;
  roboclaw1.SetEncM1(address1, 0);
  roboclaw1.SetEncM1(address2, 0);
  roboclaw1.SetEncM1(address3, 0);
  lengthOfRobot = 0;
  externalData = "";
  delay(100);
  while (logic == 0)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    CalculateSpeed(x);
    enc1 = roboclaw1.ReadEncM1(address1);
    enc2 = roboclaw1.ReadEncM1(address2);
    enc3 = roboclaw1.ReadEncM1(address3);
    roboclaw1.SpeedM1(address1, (int)(Va_base));
    roboclaw1.SpeedM1(address2, (int)(Vb_base));
    roboclaw1.SpeedM1(address3, (int)(Vc_base));
    lengthOfRobot = (float)(0.6667) * (enc1 * angleA + enc2 * angleB + enc3 * angleC);
    String data = uart();
    char* json = data.c_str();
    DeserializationError error = deserializeJson(doc, json);
    if (!error)
    {
      externalData = data;
      logic = 2;
    }
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

int decoderOfOperation(int status)
{
  if (status == 0)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    String send = "";
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
    send += String(x, DEC);
    send += '"';
    send += '}';
    Serial.println(send);
    delay(50);
    resetFunc();
  }
  else if (status == 1)
  {
    String angleString = doc[String("a")].as<String>();
    String lengthString = doc[String("b")].as<String>();
    int movingStaus = move(angleString.toInt(), lengthString.toInt());
    delay(50);
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    String send = "";
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
    send += String(x, DEC);
    send += '"';
    send += '}';
    Serial.println(send);
    if(movingStaus == 2)
    {
      decoderOfOperation(decoder(externalData));
    }
  }
  else if (status == 2)
  {

  }
  else if (status == 3)
  {
    String angleString = doc[String("a")].as<String>();
    rotationWithPID(angleString.toInt());
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    String send = "";
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
    send += String(x, DEC);
    send += '"';
    send += '}';
    Serial.println(send);
  }
  else if (status == 4)
  {
     String angleString = doc[String("a")].as<String>();
     
  }
  else if (status == 5)
  {

  }
  else if (status == 6)
  {

  }
}
