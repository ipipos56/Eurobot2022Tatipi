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

int staticAngle = 0;
float lengthOfRobot = 0;
float alpha = 0;
float angleA, angleB, angleC;
float Va_base = 0, Vb_base = 0, Vc_base = 0;
long int enc1 = 0, enc2 = 0, enc3 = 0;
extern volatile unsigned long timer0_millis;

void(* resetFunc) (void) = 0;
void CalculateSpeed(int _angle);
void move(int angle, long int length);
int decoder(String _data);
String uart();
void decoderOfOperation(int status);

void setup() {
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
void loop() {
  decoderOfOperation(decoder(uart()));
  while (1);
  //
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

void move(int angle, long int length)
{
  int logic = 0;
  roboclaw1.SetEncM1(address1, 0);
  roboclaw1.SetEncM1(address2, 0);
  roboclaw1.SetEncM1(address3, 0);
  lengthOfRobot = 0;
  delay(100);
  while (logic == 0)
  {
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    int x = euler.x();
    CalculateSpeed(x);
    Serial.println(x);
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
      logic = 2;
    }
    if (lengthOfRobot >= length)
    {
      logic = 1;
    }
    delay(1);
  }
  //Serial.println("pridurok");
  roboclaw1.SpeedM1(address1, 0);
  roboclaw1.SpeedM1(address2, 0);
  roboclaw1.SpeedM1(address3, 0);
}

void decoderOfOperation(int status)
{

  if (status == 0)
  { imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
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
    Serial.println("Ya");
    delay(50);
    String angleString = doc[String("a")].as<String>();
    String lengthString = doc[String("l")].as<String>();
    move(angleString.toInt(), lengthString.toInt());
  }
}
