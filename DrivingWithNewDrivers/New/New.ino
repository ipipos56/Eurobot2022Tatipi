#include <SoftwareSerial.h>
#include "RoboClaw.h"

SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 4600);

#define address1 0x80
#define address2 0x81

#define pi PI

#define R 0.058/2 * 100
#define _R 0.16 * 100
#define Radius 0.32 * 100
#define speed 160

const int Tick = 2000;

int32_t enc1;
int32_t enc2;
int32_t enc3;

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

int velocity = 20;
int EncMaxNum = 0;
float dt = 50;
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

float kpA = 1, kpB = 1, kpC = 1;
float kiA = 0.001, kiB = 0.001, kiC = 0.001;
float kdA = 0.05, kdB = 0.05, kdC = 0.05;

void SyncMove()
{
  uint8_t status1, status2, status3;
  bool valid1, valid2, valid3;
  enc1 = roboclaw.ReadEncM1(address1, &status1, &valid1);
  enc2 = roboclaw.ReadEncM2(address2, &status2, &valid2);
  enc3 = roboclaw.ReadEncM2(address1, &status3, &valid3);
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
  Va = Va_base + Ua;
  Vb = Vb_base + Ub;
  Vc = Vc_base + Uc;
  if (Va > 0)    roboclaw.ForwardM1(address1, (int)abs(Va));
  else  roboclaw.BackwardM1(address1, (int)abs(Va));
  if (Vb > 0)    roboclaw.ForwardM2(address2, (int)abs(Vb));
  else  roboclaw.BackwardM2(address2, (int)abs(Vb));
  if (Vc > 0)    roboclaw.ForwardM2(address1, (int)abs(Vc));
  else  roboclaw.BackwardM2(address1, (int)abs(Vc));
  //  if (Va_base > 0)    roboclaw.ForwardM1(address1, abs(Va_base));
  //  else  roboclaw.BackwardM1(address1, abs(Va_base));
  //  if (Vb_base > 0)    roboclaw.ForwardM2(address2, abs(Vb_base));
  //  else  roboclaw.BackwardM2(address2, abs(Vb_base));
  //  if (Vc_base > 0)    roboclaw.ForwardM2(address1, abs(Vc_base));
  //  else  roboclaw.BackwardM2(address1, abs(Vc_base));
  Serial.print("Encoder1:" + String(errA) + "\t");
  Serial.print("Encoder2:" + String(errB) + "\t");
  Serial.println("Encoder3:" + String(errC));
  //  Serial.print("Encoder1:" + String(Va) + "\t");
  //  Serial.print("Encoder2:" + String(Vb) + "\t");
  //  Serial.println("Encoder3:" + String(Vc));
}

void setup()
{
  Serial.begin(57600);
  roboclaw.begin(38400);
}
void loop()
{

  velocity = 50;
  resetAll();
  alpha = PI / 4;
  CalculateSpeed();
  CalculateKoef();
  roboclaw.SetEncM1(address1, 0);
  roboclaw.SetEncM2(address2, 0);
  roboclaw.SetEncM2(address1, 0);
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  roboclaw.SpeedM2(address1, 0);
  delay(2000);
  while(1);
  while (1)
  {
    SyncMove();
  }
  Serial.println("lol");
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  while (1);
}
