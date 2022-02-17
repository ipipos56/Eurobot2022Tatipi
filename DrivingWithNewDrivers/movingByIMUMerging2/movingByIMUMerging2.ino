#include <ArduinoJson.h>
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Servo.h>
#include <EncButton.h>
#define EB_BETTER_ENC

Servo myservo12;
Servo myservo23;
Servo myservo31;
SoftwareSerial serial(10, 11);
RoboClaw roboclaw(&serial, 10000);
DynamicJsonDocument doc(2048);

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29);


#define address1 0x80
#define address2 0x81

#define pi PI

#define R 0.058/2 * 100
#define _R 0.16 * 100
#define Radius 0.32 * 100

uint32_t myTimer1;
int period = 100;

int enc1;
int enc2;
int enc3;
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
float kiEncA = 0.00001, kiEncB = 0.00001, kiEncC = 0.00001;
float kdEncA = 0.01, kdEncB = 0.01, kdEncC = 0.01;

int direction = 12;

void RotationForHex(int settingDegree, int _dir)
{
  Va_base = 0;
  Vb_base = 0;
  Vc_base = 0;
  while (1)
  {

    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    x = ((((int)euler.x() + 180) % 360) + correction) % 360;
    switch (_dir)
    {
      case 12:
        if ((x - settingDegree) < 0)
        {
          Va_base = 0;
          Vb_base = 0;
          Vc_base = -30;
        }

        else
        {

          Va_base = 0;
          Vb_base = 0;
          Vc_base = 30;
        }
        break;
      case 23:
        if ((x - settingDegree) < 0)
        {
          Va_base = 0;
          Vc_base = 0;
          Va_base = -30;
        }
        else
        {
          Va_base = 0;
          Vc_base = 0;
          Va_base = 30;
        }
        break;
      case 31:
        if ((x - settingDegree) < 0)
        {
          Va_base = 0;
          Vc_base = 0;
          Vb_base = -30;
        }
        else
        {
          Va_base = 0;
          Vc_base = 0;
          Vb_base = 30;
        }
        break;
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
      Va_base = -30;
      Vb_base = -30;
      Vc_base = -30;
    }
    else
    {
      Va_base = 30;
      Vb_base = 30;
      Vc_base = 30;
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
}



void pidForMotor()
{
  roboclaw.SpeedM1(address2, 0);
  delay(100);
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  int SettingXDegrees = ((int)euler.x() + 180) % 360;
  //  Serial.println(SettingXDegrees);
  velocity = 0;
  CalculateSpeed();
  CalculateKoef();
  resetAll();
  kpEncA = 0.6, kpEncB = 0.6, kpEncC = 0.6;
  kiEncA = 0, kiEncB = 0, kiEncC = 0;
  kdEncA = 0.3, kdEncB = 0.3, kdEncC = 0.3;
  float precent = 0.8;
  kpA = 0.4;
  kdA = 0.2;
  kiA = 0;
  kpB = 0.4;
  kdB = 0.2;
  kiB = 0;
  kpC = 0.4;
  kdC = 0.2;
  kiC = 0;
  dt = 10;
  roboclaw.SpeedM1(address1, 0);
  roboclaw.SpeedM2(address2, 0);
  roboclaw.SpeedM2(address1, 0);
  String data;
  uint8_t status1, status2, status3;
  uint8_t _status1, _status2, _status3;
  bool valid1, valid2, valid3;
  bool _valid1, _valid2, _valid3;
  bool gettingCorrection = 0;
  int speed1, speed2, speed3;
  int i = 0;
  int previousDir = 12;
  correction = 0;
  xx = SettingXDegrees;
  //Serial.println("Hello");
  String statusMes = "";
  String message = "";
  String stopMotors = "";
  String dir = "";
  String angleString = "";
  String _StringTick = "";
  while (1)
  {
    if (millis() - myTimer1 >= period) {   // ищем разницу (500 мс)
      myTimer1 += period;                  // сброс таймера
      imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      x = ((((int)euler.x() + 180) % 360) + correction) % 360;
      //Serial.println(x);
    }
    if (Serial.available())
    {
      data = Serial.readStringUntil('\n');
      //delay(100);
      Serial.println(data);
      char* json = data.c_str();
      deserializeJson(doc, json);
      statusMes = doc[String("s")].as<String>();
      message = "Good";
      stopMotors = doc["m"].as<String>();
      _StringTick = doc["t"].as<String>();
      dir = doc["d"].as<String>();
      angleString = doc["a"].as<String>();
      //Serial.println(String(statusMes) + "\t" + String(stopMotors));
      if ((statusMes == "1") && (stopMotors == "0"))
      {
        //Serial.println(String(stopMotors) + " 1");
        direction = dir.toInt();
        roboclaw.SpeedM1(address1, 0);
        roboclaw.SpeedM2(address2, 0);
        roboclaw.SpeedM2(address1, 0);
        double angleDouble = angleString.toInt();
        resetAll();
        alpha = angleDouble * PI / 180;
        velocity = 60;
        CalculateSpeed();
        CalculateKoef();
        Serial.println(message + "Da");
      }
      else if ((statusMes == "1") and (stopMotors == "1"))
      {
        //Serial.println(String(stopMotors) + " 1");
        
        velocity = 0;
        resetAll();
        CalculateSpeed();
        CalculateKoef();
        delay(10);
        //{"status":"1","message":"Good","stop":"1","angle":"45"}
        //{"s":"1","m":"0","a":"45","d":"12","t":"-150"}
        //{"st":"4","mes":"Good","stop":"0","an":"45","dir":"12","t":"-150"}
        Serial.println(message);
      }
      else if ((statusMes == "2") and (stopMotors == "0"))
      {
        //Serial.println(String(stopMotors) + " 2");
        int angleDouble = angleString.toInt();
        if (angleDouble < 0)
          angleDouble = (360 + angleDouble) % 360;
        roboclaw.SpeedM1(address1, 0);
        roboclaw.SpeedM2(address2, 0);
        roboclaw.SpeedM2(address1, 0);
        correction = angleDouble;
        //Rotation((x + correction) % 360);
        resetAll();
        velocity = 0;
        CalculateSpeed();
        CalculateKoef();
        Serial.println(message);
        //correction = 0;
      }
      else if ((statusMes == "3") and (stopMotors == "0"))
      {
        //Serial.println(String(stopMotors) + " 3");
        int angleDouble = angleString.toInt();
        if (angleDouble < 0)
          angleDouble = (360 + angleDouble) % 360;
        roboclaw.SpeedM1(address1, 0);
        roboclaw.SpeedM2(address2, 0);
        roboclaw.SpeedM2(address1, 0);
        correction = angleDouble;
        resetAll();
        velocity = 0;
        CalculateSpeed();
        CalculateKoef();
        Serial.println(message);
      }
      else if ((statusMes = "4") and (stopMotors == "0"))
      {
        //Serial.println(String(stopMotors) + " 4");
        roboclaw.BackwardM1(address2, 30);
        int angleDouble = angleString.toInt();
        int _direction = dir.toInt();
        int _Tick = _StringTick.toInt();
        Serial.println(message);
      }
      else if ((statusMes = "4") && (stopMotors == "1"))
      {
        //Serial.println("HUI");
        roboclaw.SpeedM1(address2, 0);
        int angleDouble = angleString.toInt();
        int _direction = dir.toInt();
        int _Tick = _StringTick.toInt();
        Serial.println(message);
      }

    }
//    if ((abs(SettingXDegrees - x)) >= 2)
//    {
//      Serial.println(x);
//      if (statusMes == "3")
//      {
//        RotationForHex(SettingXDegrees, direction);
//      }
//      else
//        Rotation(SettingXDegrees);
//      resetAll();
//      if (statusMes == "1" and stopMotors == "0")
//      {
//        CalculateSpeed();
//        CalculateKoef();
//        velocity = 80;
//      }
//      else
//      {
//        velocity = 0;
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
    //Serial.println(String(enc1) + "\t" + String(enc2) + "\t" + String(enc3));
    if ((abs(enc1) > 2000) || (abs(enc2) > 2000) || (abs(enc3) > 2000)) resetAll();
    switch (direction)
    {
      case 12:
        speed1 = roboclaw.ReadSpeedM1(address1, &_status1, &_valid1);
        speed2 = roboclaw.ReadSpeedM2(address2, &_status2, &_valid2);
        speed3 = roboclaw.ReadSpeedM2(address1, &_status3, &_valid3);
        speed1 = map(speed1, - 5000, 5000, -129, 129);
        speed2 = map(speed2, - 5000, 5000, -129, 129);
        speed3 = map(speed3, - 5000, 5000, -129, 129);
        enc1 = roboclaw.ReadEncM1(address1, &status1, &valid1);
        enc2 = roboclaw.ReadEncM2(address2, &status2, &valid2);
        enc3 = roboclaw.ReadEncM2(address1, &status3, &valid3);
        break;
      case 23:
        speed1 = roboclaw.ReadSpeedM2(address2, &_status1, &_valid1);
        speed2 = roboclaw.ReadSpeedM2(address1, &_status2, &_valid2);
        speed3 = roboclaw.ReadSpeedM1(address1, &_status3, &_valid3);
        speed1 = map(speed1, - 5000, 5000, -129, 129);
        speed2 = map(speed2, - 5000, 5000, -129, 129);
        speed3 = map(speed3, - 5000, 5000, -129, 129);
        enc1 = roboclaw.ReadEncM2(address2, &status1, &valid1);
        enc2 = roboclaw.ReadEncM2(address1, &status2, &valid2);
        enc3 = roboclaw.ReadEncM1(address1, &status3, &valid3);
        break;
      case 31:
        speed1 = roboclaw.ReadSpeedM2(address1, &_status1, &_valid1);
        speed2 = roboclaw.ReadSpeedM1(address1, &_status2, &_valid2);
        speed3 = roboclaw.ReadSpeedM2(address2, &_status3, &_valid3);
        speed1 = map(speed1, - 5000, 5000, -129, 129);
        speed2 = map(speed2, - 5000, 5000, -129, 129);
        speed3 = map(speed3, - 5000, 5000, -129, 129);
        enc1 = roboclaw.ReadEncM2(address1, &status1, &valid1);
        enc2 = roboclaw.ReadEncM1(address1, &status2, &valid2);
        enc3 = roboclaw.ReadEncM2(address2, &status3, &valid3);
        break;
    }
    if (abs(enc1) > 10000 || abs(enc2) > 10000 || abs(enc3) > 10000) resetAll();
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
      Da = (errA - prevErrA) * dt;
      Db = (errB - prevErrB) * dt;
      Dc = (errC - prevErrC) * dt;
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
      Ua = kpA * errA + Da * kdA;
      Ub = kpB * errB + Db * kdB;
      Uc = kpC * errC + Dc * kdC;
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
      switch (EncMaxNum)
      {
        case 1:

          Va = (int)(Va_base + Ua);
          Vb = (int)(Vb_base + UEncB * precent + Ub * (float)(1.0 - precent));
          Vc = (int)(Vc_base + UEncC * precent + Uc * (float)(1.0 - precent));
          break;
        case 2:

          Va = (int)(Va_base + UEncA * precent + Ua * (float)(1.0 - precent));
          Vb = (int)(Vb_base + Ub);
          Vc = (int)(Vc_base + UEncC * precent + Uc * (float)(1.0 - precent));
          break;
        case 3:

          Va = (int)(Va_base + UEncA * precent + Ua * (float)(1.0 - precent));
          Vb = (int)(Vb_base + UEncB * precent + Ub * (float)(1.0 - precent));
          Vc = (int)(Vc_base + Uc);
          break;
      }
      //      Va = (int)(Va_base + UEncA * precent + Ua * (float)(1.0 - precent));
      //      Vb = (int)(Vb_base + UEncB * precent + Ub * (float)(1.0 - precent));
      //      Vc = (int)(Vc_base + UEncC * precent + Uc * (float)(1.0 - precent));
      switch (direction)
      {
        case 12:
          if (Va > 0)    roboclaw.ForwardM1(address1, abs(Va));
          else  roboclaw.BackwardM1(address1, abs(Va));
          if (Vb > 0)    roboclaw.ForwardM2(address2, abs(Vb));
          else  roboclaw.BackwardM2(address2, abs(Vb));
          if (Vc > 0)    roboclaw.ForwardM2(address1, abs(Vc));
          else  roboclaw.BackwardM2(address1, abs(Vc));
          break;
        case 23:
          if (Va > 0)    roboclaw.ForwardM2(address2, abs(Va));
          else  roboclaw.BackwardM2(address2, abs(Va));
          if (Vb > 0)    roboclaw.ForwardM2(address1, abs(Vb));
          else  roboclaw.BackwardM2(address1, abs(Vb));
          if (Vc > 0)    roboclaw.ForwardM1(address1, abs(Vc));
          else  roboclaw.BackwardM1(address1, abs(Vc));
          break;
        case 31:
          if (Va > 0)    roboclaw.ForwardM2(address1, abs(Va));
          else  roboclaw.BackwardM2(address1, abs(Va));
          if (Vb > 0)    roboclaw.ForwardM1(address1, abs(Vb));
          else  roboclaw.BackwardM1(address1, abs(Vb));
          if (Vc > 0)    roboclaw.ForwardM2(address2, abs(Vc));
          else  roboclaw.BackwardM2(address2, abs(Vc));
          break;
      }
      //Serial.println(String(enc1) + "\t" + String(enc2) + "\t" + String(enc3));
    }
    else if ((abs(speed1) > 5000) || (abs(speed2) > 5000)  || (abs(speed3) > 5000) )
    {
      resetAll();
      //Serial.println("error");
    }
    else
    {
      resetAll();
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
}

void loop()
{
  //roboclaw.ForwardM2(address2, 60);
  //  Serial.println("Hello");
  //  roboclaw.BackwardM1(address2, 50);
  //  delay(500);
  //  roboclaw.ForwardM1(address2, 0);
  //  UpDownLICK(-350, 0, 12);
  //  servoRotation(90,23);
  //roboclaw.ForwardM1(address2, 0)
  //roboclaw.SpeedM1(address2, 0);
  pidForMotor();

}
