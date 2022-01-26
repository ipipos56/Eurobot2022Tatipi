#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"

SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 10000);
DynamicJsonDocument doc(2048);

#define address1 0x80
#define address2 0x81

#define pi PI

#define R 0.058/2 * 100
#define _R 0.16 * 100
#define Radius 0.32 * 100
#define speed 160

const int Tick = 1500;

uint8_t status1, status2, status3;
bool valid1, valid2, valid3;
bool stops = false;
int32_t enc1;
int32_t enc2;
int32_t enc3;

float kpA = 0.05, kpB = 0.05, kpC = 0.05;
float kiA = 0.001, kiB = 0.001, kiC = 0.001;
float kdA = 0.5, kdB = 0.5, kdC = 0.5;

bool _finishMoving = 0;

float angleOfRobot = 0;
float prevErrA, prevErrB, prevErrC;
float integralA = 0, integralB = 0, integralC = 0;
float Da, Db, Dc;
float alpha = 0;
float Va = 0;
float Vb = 0;
float Vc = 0;
float Va_base = 0;
float Vb_base = 0;
float Vc_base = 0;
float Vmax = 0;
float Ma = 1;
float Mb = 1;
float Mc = 1;

int velocity = 40;
int EncMaxNum = 0;
float dt = 100;
int maxEnc = 0;

int tickA = 0;
int tickB = 0;
int tickC = 0;

float errA = 0;
float errB = 0;
float errC = 0;
float Ua = 0;
float Ub = 0;
float Uc = 0;

void resetAll()
{
  maxEnc = 0;
  errA = 0;
  errB = 0;
  errC = 0;
  prevErrA = 0;
  prevErrB = 0;
  prevErrC = 0;
  Ua = 0;
  Ub = 0;
  Uc = 0;
  Da = 0;
  Db = 0;
  Dc = 0;
  integralA = 0;
  integralB = 0;
  integralC = 0;
}

void stopMotor()
{
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM2(address2, 0);
}

void CalculateSpeed()
{
  Va_base = velocity * cos((150 * PI / 180 - alpha));
  Vb_base = velocity * cos((30 * PI / 180 - alpha));
  Vc_base = velocity * cos((270 * PI / 180 - alpha));
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
  if ((Va_base != 0) && (Vmax != 0))
  {
    Ma = Va_base / Vmax;
  }
  else
  {
    Ma = 0;
  }
  if ((Vb_base != 0) && (Vmax != 0))
  {
    Mb = Vb_base / Vmax;
  }
  else
  {
    Mb = 0;
  }
  if ((Vc_base != 0) && (Vmax != 0))
  {
    Mc = Vc_base / Vmax;
  }
  else
  {
    Mc = 0;
  }
}
float sumErr = 0;


void messageCheck()
{
  bool checked = false;
  while(!checked) {
    String data = Serial.readStringUntil('\n');

    char* json = data.c_str();

    deserializeJson(doc, json);
    String statusMes = doc["status"].as<String>();
    String message = doc["message"].as<String>();
    String stopMotors = doc["stop"].as<String>();
    if(statusMes == "1" and stopMotors == "0")
    {
      checked = true;
      Serial.println(message);
      String angleString =  doc["angle"];
      double angleDouble = angleString.toDouble();
    }
    else if(statusMes == "1" and stopMotors == "1")
    {
      checked = true;
      stopMotor();
      //{"status":"1","message":"Good","stop":"0","angle":"2.3"}
    }
    else
      Serial.println("Error in sending: " +  statusMes + " " + stopMotors);
  }
}



void SyncMove()
{
  status1, status2, status3;
  valid1, valid2, valid3;
  enc1 = roboclaw.ReadEncM2(address1, &status1, &valid1);
  enc2 = roboclaw.ReadEncM1(address1, &status2, &valid2);
  enc3 = roboclaw.ReadEncM2(address2, &status3, &valid3);
  if (max(abs(enc1), max(abs(enc2), abs(enc3))) > 30000)
  {
    roboclaw.ResetEncoders(address2);
    roboclaw.ResetEncoders(address1);
    enc1 = roboclaw.ReadEncM2(address1, &status1, &valid1);
    enc2 = roboclaw.ReadEncM1(address1, &status2, &valid2);
    enc3 = roboclaw.ReadEncM2(address2, &status3, &valid3);
  }
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
  errA = Ma * maxEnc - enc1;
  errB = Mb * maxEnc - enc2;
  errC = Mc * maxEnc - enc3;
  integralA = integralA + errA * dt;
  integralB = integralB + errB * dt;
  integralC = integralC + errC * dt;
  Da = (errA - prevErrA) / dt;
  Db = (errB - prevErrB) / dt;
  Dc = (errC - prevErrC) / dt;
  prevErrA = errA;
  prevErrB = errB;
  prevErrC = errC;
  Ua = kpA * errA;
  Ub = kpB * errB;
  Uc = kpC * errC;
  Va = Ua + Va_base + Da * kdA;
  Vb = Ub + Vb_base + Db * kdB;
  Vc = Uc + Vc_base + Dc * kdC;
  if (Va > 0)
  {
    roboclaw.ForwardM2(address1, abs(Va));
  }
  else
  {
    roboclaw.BackwardM2(address1, abs(Va));
  }
  if (Vb > 0)
  {
    roboclaw.ForwardM1(address1, abs(Vb));
  }
  else
  {
    roboclaw.BackwardM1(address1, abs(Vb));
  }
  if (Vc > 0)
  {
    roboclaw.ForwardM2(address2, abs(Vc));
  }
  else
  {
    roboclaw.BackwardM2(address2, abs(Vc));
  }
}


void setup()
{
  
  Serial.begin(115200);
  roboclaw.begin(38400);
}
void loop()
{

  delay(1000);
  float distance = 20;
  float _distance = abs((distance * Tick) / (2 * PI * R));
  resetAll();
  alpha = PI / 4;
  CalculateSpeed();
  CalculateKoef();
  roboclaw.ResetEncoders(address1);
  roboclaw.ResetEncoders(address2);
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  while (1)
  {
    if (abs(cos(150 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 1))
      break;
    if (abs(cos(30 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 2))
      break;
    if (abs(cos(270 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 3))
      break;
    SyncMove();
  }
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  delay(500);

  resetAll();
  alpha = 3 * PI / 4;
  CalculateSpeed();
  CalculateKoef();
  roboclaw.ResetEncoders(address1);
  roboclaw.ResetEncoders(address2);
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  while (1)
  {
    if (abs(cos(150 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 1))
      break;
    if (abs(cos(30 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 2))
      break;
    if (abs(cos(270 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 3))
      break;
    SyncMove();
  }
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  delay(500);
  resetAll();
  alpha = 3 * PI / 4 + PI/2;
  CalculateSpeed();
  CalculateKoef();
  roboclaw.ResetEncoders(address1);
  roboclaw.ResetEncoders(address2);
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  while (1)
  {
    if (abs(cos(150 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 1))
      break;
    if (abs(cos(30 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 2))
      break;
    if (abs(cos(270 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 3))
      break;
    SyncMove();
  }
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  delay(500);
  resetAll();
  alpha = -PI/4;
  CalculateSpeed();
  CalculateKoef();
  roboclaw.ResetEncoders(address1);
  roboclaw.ResetEncoders(address2);
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  while (1)
  {
    if (abs(cos(150 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 1))
      break;
    if (abs(cos(30 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 2))
      break;
    if (abs(cos(270 * PI / 180 - alpha) * abs(maxEnc)) > _distance && (EncMaxNum == 3))
      break;
    SyncMove();
  }
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  while (1);
  
}
