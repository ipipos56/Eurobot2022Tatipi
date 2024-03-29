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

int32_t enc1;
int32_t enc2;
int32_t enc3;

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
float Va_base = 0;
float Vb_base = 0;
float Vc_base = 0;
float Vmax = 0;
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
//float kiEncA = 0.00001, kiEncB = 0.00001, kiEncC = 0.00001;
float kdEncA = 0.01, kdEncB = 0.01, kdEncC = 0.01;

void pidForMotor()
{
  alpha = 45 * pi / 180;
  velocity = 0;
  CalculateSpeed();
  CalculateKoef();
  resetAll();
  kpEncA = 0.5, kpEncB = 0.5, kpEncC = 0.5;
  kdEncA = 0.005, kdEncB = 0.005, kdEncC = 0.005;
  kpA = 1.2;
  kdA = 4;
  kiA = 0;
  kpB = 1.2;
  kdB = 4;
  kiB = 0;
  kpC = 1.2;
  kdC = 4;
  kiC = 0;
  dt = 100;
  float precent = 1;
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  roboclaw.SpeedM2(address1, 0);
  delay(2000);
  String data;
  uint8_t status1, status2, status3;
  bool valid1, valid2, valid3;
  uint8_t _status1, _status2, _status3;
  bool _valid1, _valid2, _valid3;
  int speed1, speed2, speed3;
  while (1)
  {
    //    if (Serial.available())
    //    {
    //      data = Serial.readStringUntil('\n');
    //      //Serial.println(data);
    //      char* json = data.c_str();
    //      deserializeJson(doc, json);
    //      String statusMes = doc["status"].as<String>();
    //      String message = doc["message"].as<String>();
    //      String stopMotors = doc["stop"].as<String>();
    //
    //      if ((statusMes == "1") && (stopMotors == "0"))
    //      {
    //        //Serial.println(message);
    //        roboclaw.SpeedM1(address1, 0);
    //        roboclaw.SpeedM2(address2, 0);
    //        roboclaw.SpeedM2(address1, 0);
    //        String angleString =  doc["angle"];
    //        double angleDouble = angleString.toInt();
    //        resetAll();
    //        alpha = angleDouble * PI / 180;
    //        velocity = 50;
    //        CalculateSpeed();
    //        CalculateKoef();
    //        //Serial.println(velocity);
    //      }
    //      else if (statusMes == "1" and stopMotors == "1")
    //      {
    //        velocity = 0;
    //        CalculateSpeed();
    //        CalculateKoef();
    //        resetAll();
    //
    //        //{"status":"1","message":"Good","stop":"0","angle":"2.3"}
    //        //{"status":"1","message":"Good","stop":"1","angle":"45"}
    //        //{"status":"1","message":"Good","stop":"0","angle":"45","velocity":"50"}
    //      }
    //
    //    }
    speed1 = roboclaw.ReadSpeedM1(address1, &_status1, &_valid1);
    speed2 = roboclaw.ReadSpeedM2(address2, &_status2, &_valid2);
    speed3 = roboclaw.ReadSpeedM2(address1, &_status3, &_valid3);
    speed1 = map(speed1, - 5000, 5000, -129, 129);
    speed2 = map(speed2, - 5000, 5000, -129, 129);
    speed3 = map(speed3, - 5000, 5000, -129, 129);
    enc1 = roboclaw.ReadEncM1(address1, &status1, &valid1);
    enc2 = roboclaw.ReadEncM2(address2, &status2, &valid2);
    enc3 = roboclaw.ReadEncM2(address1, &status3, &valid3);
    if (valid1 && valid2 && valid3 && _valid1 && _valid2 && _valid3 )
    {
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
      errA = Va_base - speed1;
      errB = Vb_base - speed2;
      errC = Vc_base - speed3;
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
      Ua = kpA * errA + Da * kdA;// + integralA * kiA);
      Ub = kpB * errB + Db * kdB;// + integralB * kiB);
      Uc = kpC * errC + Dc * kdC;// + integralC * kiC);
      UEncA = (kpEncA * errEncA + DEncA * kdEncA);// + integralEncA * kiEncA;
      UEncB = (kpEncB * errEncB + DEncB * kdEncB);//+ integralEncB * kiEncB;
      UEncC = (kpEncC * errEncC + DEncC * kdEncC);// + integralEncC * kiEncC;
      //    Da = 0;
      //    Db = 0;
      //    Dc = 0;
      //    DEncA = 0;
      //    DEncB = 0;
      //    DEncC = 0;
      //    integralEncA = 0;
      //    integralEncB = 0;
      //    integralEncC = 0;
      //    integralA = 0;
      //    integralB = 0;
      //    integralC = 0;
      Va = (int)(Va_base + UEncA);
      Vb = (int)(Vb_base + UEncB);
      Vc = (int)(Vc_base + UEncC);

      if (Va > 0)    roboclaw.ForwardM1(address1, abs(Va));
      else  roboclaw.BackwardM1(address1, abs(Va));
      if (Vb > 0)    roboclaw.ForwardM2(address2, abs(Vb));
      else  roboclaw.BackwardM2(address2, abs(Vb));
      if (Vc > 0)    roboclaw.ForwardM2(address1, abs(Vc));
      else  roboclaw.BackwardM2(address1, abs(Vc));
      Serial.println(String(enc1) + "\t" + String(enc2) + "\t" + String(enc3));
    }
    else if (abs(speed1) > 5000 || abs(speed2) || abs(speed3))
    {
      roboclaw.ForwardM1(address1, 0);
      roboclaw.ForwardM2(address2, 0);
      roboclaw.ForwardM2(address1, 0);
      Serial.println("error");
    }
    else
    {
      roboclaw.ForwardM1(address1, 0);
      roboclaw.ForwardM2(address2, 0);
      roboclaw.ForwardM2(address1, 0);
      Serial.println("error");
    }
  }
}

void setup()
{
  Serial.begin(57600);
  roboclaw.begin(38400);
}
void loop()
{
  pidForMotor();
  while (1);
}
