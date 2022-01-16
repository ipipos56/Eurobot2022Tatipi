#include <EncButton.h>

#define EB_BETTER_ENC
#define pi PI

#define R 0.058/2 * 100
#define _R 0.16 * 100
#define Radius 0.32 * 100
#define speed 255

bool _finishMoving = 0;
float angleOfRobot = 0;

const unsigned int IN1 = 24;
const unsigned int IN2 = 25;
const unsigned int EN1 = 3;

const unsigned int IN3 = 22;
const unsigned int IN4 = 23;
const unsigned int EN2 = 2;

const unsigned int IN5 = 26;
const unsigned int IN6 = 27;
const unsigned int EN3 = 4;

int calculateVelocity(int _velocity)
{
  return map(_velocity, 0, 100, 180, 255);
}

float alpha = 0;
int velocity = 250;
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
int EncMaxNum = 0;
int dt = 5;
int moveTime = 2000;
int moveEnc = 2000;
int maxEnc = 0;

int dirA = 0;
int dirB = 0;
int dirC = 0;

int pwmA = 0;
int pwmB = 0;
int pwmC = 0;

bool m1;
bool m2;
bool m3;

int tickA = 0;
int tickB = 0;
int tickC = 0;
float errA = 0;
float errB = 0;
float errC = 0;
float Ua = 0;
float Ub = 0;
float Uc = 0;

extern volatile unsigned long timer0_millis;


EncButton<EB_CALLBACK, 18, 19> enc1;
EncButton<EB_CALLBACK, 16, 17> enc2;
EncButton<EB_CALLBACK, 14, 15> enc3;


void CalculateSpeed()
{
  Va_base = velocity * cos((150 * PI / 180 - alpha));
  Vb_base = velocity * cos((30 * PI / 180 - alpha));
  Vc_base = velocity * cos((270 * PI / 180 - alpha));
  dirA = Va_base / abs(Va_base);
  dirB = Vb_base / abs(Vb_base);
  dirC = Vc_base / abs(Vc_base);
  pwmA = calculateVelocity(abs(Va_base));
  pwmB = calculateVelocity(abs(Vb_base));
  pwmC = calculateVelocity(abs(Vc_base));
  //  Serial.println("dir1: " + String(dirA));
  //  Serial.println("dir2: " + String(dirB));
  //  Serial.println("dir3: " + String(dirC));
  //  Serial.println("pwm: " + String(pwmA) + "\t" + String(pwmB) + "\t" + String(pwmC));
  //  Serial.println("veloc: " + String(Va_base) + "\t" + String(Vb_base) + "\t" + String(Vc_base));
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

void SyncMove()
{
  enc1.tick();
  enc2.tick();
  enc3.tick();
  float kp = 10;
  if (m1)
  {
    tickA = abs(enc1.counter);
    if (EncMaxNum == 1)
    {
      maxEnc = abs(tickA);
    }

    m1 = !m1;
    //Serial.println("veloc: " + String(Ua) + "\t" + String(Ub) + "\t" + String(Uc));
    Serial.println("ErrA: " + String(errA) + "\t" + "ErrB: " + String(errB) + "\t" + "ErrC: " + String(errC) + "\n");
  }
  if (m2)
  {
    tickB = abs(enc2.counter);
    if (EncMaxNum == 2)
    {
      maxEnc = abs(tickB);
    }
    m2 = !m2;
    //Serial.println("veloc: " + String(Ua) + "\t" + String(Ub) + "\t" + String(Uc));
    Serial.println("ErrA: " + String(errA) + "\t" + "ErrB: " + String(errB) + "\t" + "ErrC: " + String(errC) + "\n");
  }
  if (m3)
  {
    tickC = abs(enc3.counter);
    if (EncMaxNum == 3)
    {
      maxEnc = abs(tickC);
    }
    m3 = !m3;
    //Serial.println("veloc: " + String(Ua) + "\t" + String(Ub) + "\t" + String(Uc));
    Serial.println("ErrA: " + String(errA) + "\t" + "ErrB: " + String(errB) + "\t" + "ErrC: " + String(errC) + "\n");
  }
  errA = 1 - Ma - (float)(maxEnc - tickA)/maxEnc;
  Ua = kp * errA;
  Va = Ua + Va_base;
  errB = 1 - Mb - (float)(maxEnc - tickB)/maxEnc;
  Ub = kp * errB;
  Vb = Ub + Vb_base;
  errC = 1 - Mc - (float)(maxEnc - tickC)/maxEnc;
  Uc = kp * errC;
  Vc = Uc + Vc_base;
  analogWrite(EN1, (int)abs(Va));
  analogWrite(EN2, (int)abs(Vb));
  analogWrite(EN3, (int)abs(Vc));
  if (Va > 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  if (Vb > 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  if (Vc > 0)
  {
    digitalWrite(IN5, HIGH);
    digitalWrite(IN6, LOW);
  }
  else
  {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);
  }
}

void MoveRobot()
{
  
}

void setup()
{
  enc1.counter = 0;
  enc2.counter = 0;
  enc3.counter = 0;
  attachment();
  initialization();
  Serial.begin(115200);

  //Serial.println(pwmVelocity);
}
void loop()
{

  // for (float i = 0; i < 2 * PI; i += 0.01745329251994329576923690768489)
  //{
  alpha = 35 * PI / 180;
  CalculateSpeed();
  //while(1);
  CalculateKoef();
  while (1)
  {
    SyncMove();
  }
  /*
    while (1)
    {
    SyncMove();
    }*/
  float angle = alpha * 180 / PI;
  //if ((abs(Va_base) == abs(Vb_base)) || (abs(Va_base) == abs(Vc_base)) || (abs(Vb_base) == abs(Vc_base)))
  Serial.println(String(angle) + "\t" + String(Ma) + "\t" + String(Mb) + "\t" + String(Mc) + "\t" + String(Vmax));
  // }
  //PidForMoving(3, 0, 10, 1, 2, 0.001);
  while (1);
}

void attachment()
{
  enc1.attach(TURN_HANDLER, myTurn1);
  enc2.attach(TURN_HANDLER, myTurn2);
  enc3.attach(TURN_HANDLER, myTurn3);
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

bool PidForMoving(int motor, int16_t tick1, float kp, float ki, float kd, float dt)
{
  uint32_t i = 0;
  int _motor1;
  int _motor1A;
  int _motor1B;
  int _motor2;
  int _motor2A;
  int _motor2B;
  tick1 = 0;
  enc1.counter = 0;
  enc2.counter = 0;
  enc3.counter = 0;
  int err;
  static float integral = 0, prevErr = 0;
  float D;
  uint32_t timer;
  while (1)
  {
    if (millis() - timer >= 10000)
    {
      timer = millis();
      break;
    }
    switch (motor)
    {
      case 1:
        enc1.tick();
        if (m1)
        {
          m1 = !m1;
          tick1 = enc1.counter;
        }
        break;
      case 2:
        enc2.tick();
        if (m2)
        {
          m2 = !m2;
          tick1 = enc2.counter;
        }
        break;
      case 3:
        enc3.tick();
        if (m3)
        {
          m3 = !m3;
          tick1 = enc3.counter;
        }
        break;
    }
    err = i - tick1;
    integral = integral + err * dt;
    D = (err - prevErr) / dt;
    int uw = err * kp + integral * ki + D * kd;
    switch (motor)
    {
      case 1:
        analogWrite(EN1, abs(uw));
        break;
      case 2:
        analogWrite(EN2, abs(uw));
        break;
      case 3:
        analogWrite(EN3, abs(uw));
        break;
    }
    if (uw > 0)
    {
      switch (motor)
      {
        case 1:
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          break;
        case 2:
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
          break;
        case 3:
          digitalWrite(IN5, HIGH);
          digitalWrite(IN6, LOW);
          break;
      }
    }
    else
    {
      switch (motor)
      {
        case 1:
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
          break;
        case 2:
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, HIGH);
          break;
        case 3:
          digitalWrite(IN5, LOW);
          digitalWrite(IN6, HIGH);
          break;
      }
    }
    prevErr = err;
  }
  motorStop(1);
  motorStop(2);
  motorStop(3);
  return _finishMoving;
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
