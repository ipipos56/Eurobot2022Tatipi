#include <EncButton.h>

#define EB_BETTER_ENC
#define pi PI

#define R 0.058/2 * 100
#define _R 0.16 * 100
#define Radius 0.48 * 100
#define speed 195

const int Tick = 740;
const int Tick2 = 1020;

int currentX = 0;
int currentY = 0;
int previousX = 0;
int previousY = 0;

bool _finishMoving = 0;

float angleOfRobot = 0;

float kpM = 2;
float kiM = 0.01;
float kdM = 0.01;

const float d = 0.1827; //meter
const float r = 0.058 / 2;

const unsigned int IN1 = 22;
const unsigned int IN2 = 23;
const unsigned int EN1 = 18;

const unsigned int IN3 = 24;
const unsigned int IN4 = 25;
const unsigned int EN2 = 19;

const unsigned int IN5 = 26;
const unsigned int IN6 = 27;
const unsigned int EN3 = 17;

float vectorOfRobot[3][2];
float v[2];

bool m1 = 0;
bool m2 = 0;
bool m3 = 0;
EncButton<EB_CALLBACK, 2, 3> enc1;
EncButton<EB_CALLBACK, 4, 5> enc2;
EncButton<EB_CALLBACK, 6, 7> enc3;

void setup()
{
  Serial.begin(115200);
  initialization();
  attachment();
}
void loop()
{
    while (1)
    {
      enc1.tick();
      enc2.tick();
      int tick1;
      int tick2;
      if (m2)
      {
        tick2 = enc2.counter;
        Serial.println(tick2);
        m2 = !m2;
      }
      if (m1)
      {
        tick1 = enc1.counter;
        Serial.println(enc1.counter);
        m1 = !m1;
      }
    }
  //  analogWrite(18, 170);
  //  digitalWrite(22, HIGH);
  //  digitalWrite(23, LOW);
  //  delay(1000);
  //  analogWrite(18, 0);
  //  digitalWrite(22, LOW);
  //  digitalWrite(23, LOW);
  while (1);
}
void motorStop(int numberOfMotor)
{
  switch (numberOfMotor)
  {
    case 1:
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      break;
    case 2:
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      break;
    case 3:
      digitalWrite(IN5, LOW);
      digitalWrite(IN6, LOW);
      break;
  }
}

void attachment()
{
  enc1.attach(TURN_HANDLER, myTurn1);
  enc2.attach(TURN_HANDLER, myTurn2);
  enc3.attach(TURN_HANDLER, myTurn3);
}
void myTurn1()
{
  m1 = !m1;
}
void myTurn2()
{
  m2 = !m2;
}
void myTurn3()
{
  m3 = !m3;
}
void initialization()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(EN3, OUTPUT);
}

float safe_acos(float value) {
  if (value <= -1.0) {
    return PI;
  } else if (value >= 1.0) {
    return 0;
  } else {
    return acos(value);
  }
}



float to2PI(float rad)
{
  if ((rad > 0 && rad < 2 * PI) || (rad < 0 && rad > -2 * PI))
    return rad;
  else if (rad > 2 * PI)
    return to2PI(rad - 2 * PI);
  else if (rad < 0 && rad < -2 * PI)
    return  to2PI(rad + 2 * PI);
}
