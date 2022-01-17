#include <EncButton.h>
#include <Servo.h>

#define EB_BETTER_ENC
#define pi PI

#define R 0.058/2 * 100
#define _R 0.16 * 100
#define Radius 0.32 * 100
#define speed 180

Servo myservo;

float kpA = 5, kpB = 5, kpC = 5;
float kiA = 0.0015, kiB = 0.0015, kiC = 0.0015;
float kdA = 2, kdB = 2, kdC = 2;

bool _finishMoving = 0;

const unsigned int IN1 = 24;
const unsigned int IN2 = 25;
const unsigned int EN1 = 3;

const unsigned int IN3 = 22;
const unsigned int IN4 = 23;
const unsigned int EN2 = 2;

const unsigned int IN5 = 26;
const unsigned int IN6 = 27;
const unsigned int EN3 = 4;


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

int velocity = 250;
int EncMaxNum = 0;
int dt = 0.001;
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


EncButton<EB_CALLBACK, 19, 18> enc1;
EncButton<EB_CALLBACK, 17, 16> enc2;
EncButton<EB_CALLBACK, 15, 14> enc3;


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
void SyncMove()
{

  enc1.tick();
  enc2.tick();
  enc3.tick();
  if (m1)
  {
    tickA = enc1.counter;
    if (EncMaxNum == 1)
    {
      maxEnc = tickA;
    }
    m1 = !m1;
    //Serial.println("ErrA: " + String(errA) + "\t" + "ErrB: " + String(errB) + "\t" + "ErrC: " + String(errC) + "\n");
  }
  if (m2)
  {
    tickB = enc2.counter;
    if (EncMaxNum == 2)
    {
      maxEnc = tickB;
    }
    m2 = !m2;
    //Serial.println("ErrA: " + String(errA) + "\t" + "ErrB: " + String(errB) + "\t" + "ErrC: " + String(errC) + "\n");
  }
  if (m3)
  {
    tickC = enc3.counter;
    if (EncMaxNum == 3)
    {
      maxEnc = tickC;
    }
    m3 = !m3;
    //Serial.println("ErrA: " + String(errA) + "\t" + "ErrB: " + String(errB) + "\t" + "ErrC: " + String(errC) + "\n");
  }
  errA = Ma * maxEnc - tickA;
  errB = Mb * maxEnc - tickB;
  errC = Mc * maxEnc - tickC;
  integralA = integralA + errA * dt;
  integralB = integralB + errB * dt;
  integralC = integralC + errC * dt;
  Da = (errA - prevErrA)/dt;
  Db = (errB - prevErrB)/dt;
  Dc = (errC - prevErrC)/dt;
  Ua = kpA * errA + integralA * kiA;
  Va = Ua + Va_base;
  Ub = kpB * errB + integralB* kiB;
  Vb = Ub + Vb_base;
  Uc = kpC * errC + integralC * kiC;
  Vc = Uc + Vc_base;
  analogWrite(EN1, abs((int)Va));
  analogWrite(EN2, abs((int)Vb));
  analogWrite(EN3, abs((int)Vc));
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


void setup()
{
  enc1.counter = 0;
  enc2.counter = 0;
  enc3.counter = 0;
  attachment();
  initialization();
  Serial.begin(115200);
}
void loop()
{
  //PidForMoving(1, 0, 2, 0.01, 0.02, 0.01);
  
  alpha = PI / 3;
  CalculateSpeed();
  CalculateKoef();
  while (1)
  {
    SyncMove();
  }
  Serial.println(String(Ma) + "\t" + String(Mb) + "\t" + String(Mc));
  while (1);
}

void attachment()
{
  myservo.attach(12);
  myservo.write(0); 
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
