#define EB_BETTER_ENC
#include <EncButton.h>

const unsigned int IN1 = 25;//25
const unsigned int IN2 = 24;//24
const unsigned int EN = 3;//3

const unsigned int IN3 = 23;
const unsigned int IN4 = 22;
const unsigned int EN2 = 2;

const unsigned int IN5 = 27;
const unsigned int IN6 = 26;
const unsigned int EN3 = 4;

int target = 0;
int target2 = 0;
int target3 = 0;

EncButton<EB_CALLBACK, 18, 19> enc;
EncButton<EB_CALLBACK, 16, 17> enc2;
EncButton<EB_CALLBACK, 14, 15> enc3;



void attachment()
{
  enc.attach(TURN_HANDLER, myTurn);
  enc2.attach(TURN_HANDLER, myTurn2);
  enc3.attach(TURN_HANDLER, myTurn3);
}

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN,  OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(EN3, OUTPUT);
  Serial.begin(2000000);
  attachment();
}

void motorStop(int numberOfMotor)
{
  switch (numberOfMotor)
  {
    case 0:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      break;
    case 1:
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    case 2:
      digitalWrite(IN5, LOW);
      digitalWrite(IN6, LOW);
      break;
  }
}

void speedOfMotor(int numberOfMotor, int pwm)
{
  switch (numberOfMotor)
  {
    case 0:
      analogWrite(EN, pwm);
      break;
    case 1:
      analogWrite(EN2, pwm);
      break;
    case 2:
      analogWrite(EN3, pwm);
      break;
  }
}

void motorTurn(int numberOfMotor, int rotation, int pwm)
{
  speedOfMotor(numberOfMotor, pwm);
  switch (numberOfMotor)
  {
    case 0:
      switch (rotation)
      {
        case 0:
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          break;
        case 1:
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
          break;
      }
      break;
    case 1:
     switch (rotation)
      {
        case 0:
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
          break;
        case 1:
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, HIGH);
          break;
      }
      break;
    case 2:
     switch (rotation)
      {
        case 0:
          digitalWrite(IN5, HIGH);
          digitalWrite(IN6, LOW);
          break;
        case 1:
          digitalWrite(IN5, LOW);
          digitalWrite(IN6, HIGH);
          break;
      }
      break;
  }
}

void bigDelay()
{
  delay(100000000000);
}

void myTurn() {
  Serial.println("1:" + String(enc.counter) + " : " + String(target));  // вывести счётчик
  if (target != 0)
  {
    if (target > 0)
    {
      if (target <= enc.counter)
      {
        motorStop(0);
        bigDelay();
      }
    }
    else
    {
      if (target >= enc.counter)
      {
        motorStop(0);
        bigDelay();
      }
    }
  }
}
void myTurn2() {
  Serial.println("2:" + String(enc2.counter) + " : " + String(target2));  // вывести счётчик
  if (target2 != 0)
  {
    if (target2 > 0)
    {
      if (target2 <= enc2.counter)
      {
        motorStop(1);
        bigDelay();
      }
    }
    else
    {
      if (target2 >= enc2.counter)
      {
        motorStop(1);
        bigDelay();
      }
    }
  }
}


void myTurn3() {
  Serial.println("3:" + String(enc3.counter) + " : " + String(target3));  // вывести счётчик
  if (target3 != 0)
  {
    if (target3 > 0)
    {
      if (target3 <= enc3.counter)
      {
        motorStop(2);
        bigDelay();
      }
    }
    else
    {
      if (target3 >= enc3.counter)
      {
        motorStop(2);
        bigDelay();
      }
    }
  }
}
//0 по часовой, 1 против
//при 0 энкодер положительный, при 1 отрицательный
// =============== LOOP =============
void loop() {

  target = 580;
  target2 = 580;
  target3 = 580;
  motorTurn(0, 0, 100);
  enc.tick();   // обработка всё равно здесь
  enc2.tick();
  enc3.tick();


}
