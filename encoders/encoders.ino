#define EB_BETTER_ENC
#include <EncButton.h>

#define pi PI
#define R 0.058/2 * 100

int t0 = 0;
int t1 = 0;

int uw = 0;

double a = pi / 4;
double A[3][3];
double B[3];
double x0 = 0;
double y0 = 0;
double x = 0;
double y = 0;
double t = 1;
const int Tick = 460;

const unsigned int IN1 = 25;//25
const unsigned int IN2 = 24;//24
const unsigned int EN1 = 3;//3

const unsigned int IN3 = 23;
const unsigned int IN4 = 22;
const unsigned int EN2 = 2;

const unsigned int IN5 = 27;
const unsigned int IN6 = 26;
const unsigned int EN3 = 4;

bool stop1 = false;
bool stop2 = false;
bool stop3 = false;


int target = 0;
int target2 = 0;
int target3 = 0;

EncButton<EB_CALLBACK, 18, 19> enc1;
EncButton<EB_CALLBACK, 16, 17> enc2;
EncButton<EB_CALLBACK, 14, 15> enc3;

struct robot
{
  double x;
  double y;
  double x_previous;
  double y_previous;
  double angle;
  double time;
  double w1;
  double w2;
  double w3;
  double u1;
  double u2;
  double u3;
  int tick1;
  int tick2;
  int tick3;
};

robot r;

void attachment()
{
  enc1.attach(TURN_HANDLER, myTurn);
  enc2.attach(TURN_HANDLER, myTurn2);
  enc3.attach(TURN_HANDLER, myTurn3);
}

void initialization()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN1,  OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(EN2, OUTPUT);
  pinMode(IN5, OUTPUT);
  pinMode(IN6, OUTPUT);
  pinMode(EN3, OUTPUT);
}

void setup() {
  r.x_previous = 0;
  r.y_previous = 0;
  initialization();
  Serial.begin(115200);
  attachment();
  t0 = millis();
  uw = 0;
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

/
void speedOfMotor(int numberOfMotor, int pwm)
{
  switch (numberOfMotor)
  {
    case 0:
      analogWrite(EN1, pwm);
      break;
    case 1:
      analogWrite(EN2, pwm);
      break;
    case 2:
      analogWrite(EN3, pwm);
      break;
  }
}

void motorTurn(int numberOfMotor, int pwm)
{
  int rotation;

  speedOfMotor(numberOfMotor, pwm);
  if (pwm >= 0)
    rotation = 0;
  else
    rotation = 1;
  pwm = abs(pwm);
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

bool m1 = 0;
bool m2 = 0;
bool m3 = 0;
void myTurn() {
  m1 = !m1;
}
void myTurn2() {
  m2 = !m2;
}


void myTurn3() {
  m3 = !m3;
}

//0 по часовой, 1 против
//при 0 энкодер положительный, при 1 отрицательный
// =============== LOOP =============
int speed = 180;

bool finishMoving = false;
int _tick1 = 0;
int _tick2 = 0;
bool PidForMoving(int distance, int direction, int dir, int tick1, int tick2, float kp, float ki, float kd, float dt)
{
  int _distance = (distance * Tick) / (2 * PI * R);
  int _motor1;
  int _motor1A;
  int _motor1B;
  int _motor2;
  int _motor2A;
  int _motor2B;
  enc1.tick();
  enc2.tick();
  enc3.tick();
  int err;
  static float integral = 0, prevErr = 0;
  float D;
  switch (direction)
  {
    case 12:
      _motor1 = EN1;
      _motor1A = IN1;
      _motor1B = IN2;
      _motor2 = EN2;
      _motor2A = IN3;
      _motor2B = IN4;
      if (m1)
      {
        m1 = !m1;
        tick1 = abs(enc1.counter);
        _tick1 = tick1 * cos(pi / 6);
      }
      if (m2)
      {
        m2 = !m2;
        tick2 = abs(enc2.counter);
        _tick2 = tick2 * cos(pi / 6);
      }
      if ((int)((_tick1 + _tick2) / 2) >= _distance)
        finishMoving = true;
      break;
    case 23:
      _motor1 = EN2;
      _motor1A = IN3;
      _motor1B = IN4;
      _motor2 = EN3;
      _motor2A = IN5;
      _motor2B = IN6;
      if (m2)
      {
        m2 = !m2;
        tick1 = abs(enc2.counter);
        _tick1 = tick1 * cos(pi / 6);
      }
      if (m3)
      {
        m3 = !m3;
        tick2 = abs(enc3.counter);
        _tick2 = tick2 * cos(pi / 6);
      }
      if ((int)((_tick1 + _tick2) / 2) >= _distance)
        finishMoving = true;
      break;
    case 31:
      _motor1 = EN3;
      _motor1A = IN5;
      _motor1B = IN6;
      _motor2 = EN1;
      _motor2A = IN1;
      _motor2B = IN2;
      if (m3)
      {
        m3 = !m3;
        tick1 = abs(enc3.counter);
        _tick1 = tick1 * cos(pi / 6);
      }
      if (m1)
      {
        m1 = !m1;
        tick2 = abs(enc1.counter);
        _tick2 = tick2 * cos(pi / 6);
      }
      if ((int)((_tick1 + _tick2) / 2) >= _distance)
        finishMoving = true;
      break;
  }
  err = _tick1 - _tick2;
  integral = integral + err * dt * ki;
  D = (err - prevErr) / dt;
  Serial.println(err);
  int uw = err * kp;
  if (dir < 0)
  {
    digitalWrite(_motor1A, HIGH);
    digitalWrite(_motor1B, LOW);
    digitalWrite(_motor2A, LOW);
    digitalWrite(_motor2B, HIGH);
  }
  else
  {
    digitalWrite(_motor1A, LOW);
    digitalWrite(_motor1B, HIGH);
    digitalWrite(_motor2A, HIGH);
    digitalWrite(_motor2B, LOW);
  }
  if (uw < 0)
  {
    analogWrite(_motor1, speed);
    analogWrite(_motor2, speed + uw);
  }
  else
  {
    analogWrite(_motor1, speed - uw);
    analogWrite(_motor2, speed);
  }
  prevErr = err;
  return finishMoving;
}
bool finish;
void loop() {

  finish =  PidForMoving(25, 12, 1, 0, 0, 5, 0.5, 0.5, 1);
  if (finish)
  {
    motorStop(0);
    motorStop(1);
    motorStop(2);
    Serial.println("Good");
    delay(10000000000000000);
  }
  /*
    enc1.tick();
    enc2.tick();
    if (uw > 0)
    {
    analogWrite(EN, speed);
    analogWrite(EN2, speed + uw);
    }
    else
    {
    analogWrite(EN, speed - uw);
    analogWrite(EN2, speed);
    }

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    if (m1 && m2)
    {
    m1 = !m1;
    m2 = !m2;
    uw = abs(enc.counter) - abs(enc2.counter);
    uw += 6;
    uw = constrain(uw, -40, 40);
    Serial.println(uw);
    }
    //int d = computePID(abs(enc.counter),abs(enc2.counter),1.2,0.5,1.2,0.1, 180,250);
    //Serial.println(enc.counter);
    //Serial.println(String((abs(enc.counter)-abs(enc2.counter))*1.2));
    Serial.println(String(d) + " : " + String(d2));
    analogWrite(EN, d);
    analogWrite(EN2, d2);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    //delay(2);

    delay(500);
    t0 = millis();
    bool exit = 0;
    float kp = 1.2;
    int pwm1 = 0;
    int pwm2 = 250;
    int pwm3 = 250;
    Serial.println(String(pwm1)+" : " + String(pwm2)+" : " + String(pwm3));
    int myTime = 6;
    myTime *= 1000;
    analogWrite(EN2, pwm2);
    analogWrite(EN3, pwm3);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, HIGH);

    while(exit == 0)
    {
    if((millis()-t0)>myTime)
      exit = 1;
    delay(10);
    }
    motorStop(0);
    motorStop(1);
    motorStop(2);
    bigDelay();
  */
}
