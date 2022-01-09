#define EB_BETTER_ENC
#include <EncButton.h>

#define pi PI
#define R 0.058/2

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
const int tick = 580;

const unsigned int IN1 = 25;//25
const unsigned int IN2 = 24;//24
const unsigned int EN = 3;//3

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

EncButton<EB_CALLBACK, 18, 19> enc;
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

robot calculate(robot zero)
{
  a = zero.angle;
  t = zero.time;
  x0 = zero.x_previous;
  y0 = zero.y_previous;
  x = zero.x;
  y = zero.y;

  A[0][0] = 1 / 3;
  A[0][1] = -(sqrt(3) * sin(a)) / 3;
  A[0][2] = (sqrt(3) * cos(a)) / 3;

  A[1][0] = 1 / 3;
  A[1][1] = (sqrt(3) * sin(a)) / 6 - cos(a) / 2;
  A[1][2] = -sin(a) / 2 - (sqrt(3) * cos(a)) / 6;

  A[2][0] = 1 / 3;
  A[2][1] = (sqrt(3) * sin(a)) / 6 + cos(a) / 2;
  A[2][2] = sin(a) / 2 - (sqrt(3) * cos(a)) / 6;

  B[0] = (double)0;
  B[1] = ((x - x0) * sqrt(3)) / (R * t);
  B[2] = ((y - y0) * sqrt(3)) / (R * t);

  zero.w1 = A[0][1] * B[1] + A[0][2] * B[2];
  zero.w2 = A[1][1] * B[1] + A[1][2] * B[2];
  zero.w3 = A[2][1] * B[1] + A[2][2] * B[2];

  zero.u1 = zero.w1 * R;
  zero.u2 = zero.w2 * R;
  zero.u3 = zero.w3 * R;

  zero.tick1 = zero.u1 * tick / (2 * pi * R);
  zero.tick2 = zero.u2 * tick / (2 * pi * R);
  zero.tick3 = zero.u3 * tick / (2 * pi * R);

  zero.x_previous = x;
  zero.y_previous = y;



  return zero;
}

robot r;

void attachment()
{
  enc.attach(TURN_HANDLER, myTurn);
  enc2.attach(TURN_HANDLER, myTurn2);
  enc3.attach(TURN_HANDLER, myTurn3);
}

void initialization()
{
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(EN,  OUTPUT);
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
void myTurn() {
  //Serial.println("1:" + String(enc.counter));  // вывести счётчик
  m1 = !m1;
  //uw = (abs(enc.counter) - abs(enc2.counter)) * 2;
  //Serial.println(String((abs(enc.counter) - abs(enc2.counter)) * 1.2));

}
void myTurn2() {
  //Serial.println("2:" + String(enc2.counter));  // вывести счётчик
  m2 = !m2;
  //uw = (abs(enc.counter) - abs(enc2.counter)) * 2;
  //Serial.println(String((abs(enc.counter) - abs(enc2.counter)) * 1.2));

}


void myTurn3() {
  Serial.println("3:" + String(enc3.counter));  // вывести счётчик
}

void stopper()
{
  if (stop1 == true)
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  if (stop2 == true)
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  if (stop3 == true)
  {
    digitalWrite(IN5, LOW);
    digitalWrite(IN6, LOW);
  }
}

int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) {
  int err = setpoint - input;
  static float integral = 0, prevErr = 0;
  integral = constrain(integral + (float)err * dt * ki, minOut, maxOut);
  float D = (err - prevErr) / dt;
  prevErr = err;
  Serial.println(String(err * kp + integral + D * kd));
  return constrain(err * kp + integral + D * kd, minOut, maxOut);
}
//0 по часовой, 1 против
//при 0 энкодер положительный, при 1 отрицательный
// =============== LOOP =============
int speed = 180;

void loop() {
  enc.tick();
  enc2.tick();
  if(uw>0)
  {
    analogWrite(EN, speed);
    analogWrite(EN2, speed+uw);
  }
  else
  {
    analogWrite(EN, speed-uw);
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
    uw +=6;
    uw = constrain(uw,-40,40);
    Serial.println(uw);
  }
  //int d = computePID(abs(enc.counter),abs(enc2.counter),1.2,0.5,1.2,0.1, 180,250);
  //Serial.println(enc.counter);
  //Serial.println(String((abs(enc.counter)-abs(enc2.counter))*1.2));
  /*
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
