//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include <NewPing.h>
#include "RoboClaw.h"
#define HC_TRIG 6
#define HC_ECHO 7
//See limitations of Arduino SoftwareSerial
SoftwareSerial serial(12, 13);
RoboClaw roboclaw(&serial, 10000);
NewPing sonar(HC_TRIG, HC_ECHO, 40);
#define address 0x80
const int IRpin = A1;          // аналоговый пин для подключения выхода Vo сенсора
int value1;                    // для хранения аналогового значения

void setup() {
  //Open roboclaw serial ports
  Serial.begin(38400);
  roboclaw.begin(38400);
}

void loop() {
  Serial.println("Ya");
  roboclaw.BackwardM2(address, 0);
  int cameraRotation = 0;
  delay(1000);
  while (1)
  {
    int32_t enc2 = roboclaw.ReadEncM2(address);
    // и в расстояние в см
    int distance = sonar.ping_cm();
    Serial.println(distance);
    //roboclaw.ForwardM2(address, 0); //start Motor1 forward at half speed
    if (cameraRotation == 0)
    {
      roboclaw.BackwardM2(address, 40);
    }
    else if (cameraRotation == 1)
    {
      roboclaw.BackwardM2(address, 40);
      //Serial.println(enc2);
    }
    if ((distance < 4) && (cameraRotation == 0))
    {
      roboclaw.BackwardM2(address, 0);
      delay(50);
      roboclaw.SetEncM2(address, 0);
      //enc2 = roboclaw.ReadEncM2(address);
      delay(50);
      Serial.println("First: " + String(enc2));
      cameraRotation = 1;
    }
    if ((cameraRotation == 1) && (enc2 <= -190))
    {
      roboclaw.BackwardM2(address, 0);
      delay(100);
      roboclaw.SetEncM2(address, 0);
      break;
    }
    delay(10);
    //Serial.println();
  }
  while (1);
}
