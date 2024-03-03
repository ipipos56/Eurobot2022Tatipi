#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "RoboClaw.h"

#define velocity 3000

#define address1 0x80
#define address2 0x80
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

extern volatile unsigned long timer0_millis;

SoftwareSerial serial1(10, 11);

RoboClaw roboclaw1(&serial1, 10000);

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

void(* resetFunc) (void) = 0;
void uart();

void setup()
{
  timer0_millis = UINT32_MAX - 5000;
  Serial.begin(115200);
  Serial.flush();
  roboclaw1.begin(38400);
  roboclaw1.SetM1VelocityPID(address1, Kd1, Kp1, Ki1, qpps1);
  roboclaw1.SetM2VelocityPID(address2, Kd2, Kp2, Ki2, qpps2);
  roboclaw1.SetM1VelocityPID(address3, Kd3, Kp3, Ki3, qpps3);
  roboclaw1.SetEncM1(address1, 0);
  roboclaw1.SetEncM2(address2, 0);
  roboclaw1.SetEncM1(address3, 0);
  Serial.flush();
}

void loop() {
  uart();
  delay(100);
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
      String cString = doc[String("c")].as<String>();
      int status = statusString.toInt();
      int a = aString.toInt();
      int b = bString.toInt();
      int c = cString.toInt();
      delay(100);
      Serial.println(String(status) + "\t" + String(aString) + "\t" + String(bString) + "\t" + String(cString));
      Serial.flush();
      if (status == 0)
      {  
      if (a > 0)    roboclaw1.ForwardM1(address1, abs(a));
      else  roboclaw1.BackwardM1(address1, abs(a));
      if (b > 0)    roboclaw1.ForwardM2(address2, abs(b));
      else  roboclaw1.BackwardM2(address2, abs(b));
      if (c > 0)    roboclaw1.ForwardM1(address3, abs(c));
      else  roboclaw1.BackwardM1(address3, abs(c));
      }
    }
  }
}
