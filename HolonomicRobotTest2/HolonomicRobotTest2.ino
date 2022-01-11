#include <EncButton.h>

#define EB_BETTER_ENC
#define pi PI
#define Tick 460
#define R 0.058/2 * 100
#define speed 180

const double d = 0.1827; //meter
const double r = 0.058 / 2;

const unsigned int IN1 = 25;
const unsigned int IN2 = 24;
const unsigned int EN1 = 3;

const unsigned int IN3 = 23;
const unsigned int IN4 = 22;
const unsigned int EN2 = 2;

const unsigned int IN5 = 27;
const unsigned int IN6 = 26;
const unsigned int EN3 = 4;

bool m1;
bool m2;
bool m3;

EncButton<EB_CALLBACK, 18, 19> enc1;
EncButton<EB_CALLBACK, 16, 17> enc2;
EncButton<EB_CALLBACK, 14, 15> enc3;

class Robot
{
  public:

    Robot(double x0 = 0, double y0 = 0, double x = 0, double y = 0)
    {
      _x0 = x0;
      _y0 = y0;
      _x = x;
      _y = y;
    }
    void setCurrentX(double x);
    void setCurrentY(double y);
    void setAngle(double angle);
    void firstSetVectors();
    double *get12Vector();
    double *get23Vector();
    double *get31Vector();
    void motorStop(int numberOfMotor);
    double getCurrentX();
    double getCurrentY();
    double getPreviousX();
    double getPreviousY();
    void attachment();
    void initialization();
    bool PidForMoving(int distance, int direction, int dir, int tick1, int tick2, float kp, float ki, float kd, float dt);
  private:
    int _tick1;
    int _tick2;
    bool _finishMoving;
    double _vector12[2];
    double _vector23[2];
    double _vector31[2];
    double _x0;
    double _y0;
    double _x;
    double _y;
    double _angle;
    double _A[3][3];
};
Robot firstRobot;
void setup()
{
  Serial.begin(115200);
  firstRobot.initialization();
  firstRobot.attachment();
  firstRobot.firstSetVectors();
  Serial.println(String(firstRobot.get12Vector()[0]) + "\t" + String(firstRobot.get12Vector()[1]));
  Serial.println(String(firstRobot.get23Vector()[0]) + "\t" + String(firstRobot.get23Vector()[1]));
  Serial.println(String(firstRobot.get31Vector()[0]) + "\t" + String(firstRobot.get31Vector()[1]));
}
void loop()
{
}

double *rotateVector(double *xy, double x, double y, double angle)
{
  xy[0] = x * cos(angle) - y * sin(angle);
  xy[1] = x * sin(angle) + y * cos(angle);
  return xy;
}

void Robot::firstSetVectors()
{
  _vector12[0] = 0;
  _vector12[1] = 1;

  _vector23[0] = rotateVector(_vector23, _vector12[0], _vector12[1], 2 * PI / 3)[0];
  _vector23[1] = rotateVector(_vector23, _vector12[0], _vector12[1], 2 * PI / 3)[1];

  _vector31[0] = rotateVector(_vector31, _vector12[0], _vector12[1], 4 * PI / 3)[0];
  _vector31[1] = rotateVector(_vector31, _vector12[0], _vector12[1], 4 * PI / 3)[1];

}

double *Robot::get12Vector()
{
  return _vector12;
}

double *Robot::get23Vector()
{
  return _vector23;
}

double *Robot::get31Vector()
{
  return _vector31;
}

bool Robot::PidForMoving(int distance, int direction, int dir, int tick1, int tick2, float kp, float ki, float kd, float dt)
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
        _finishMoving = true;
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
        _finishMoving = true;
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
        _finishMoving = true;
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
  return _finishMoving;
}

void Robot::motorStop(int numberOfMotor)
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
void Robot::initialization()
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
void Robot::attachment()
{
  enc1.attach(TURN_HANDLER, myTurn1);
  enc2.attach(TURN_HANDLER, myTurn2);
  enc3.attach(TURN_HANDLER, myTurn3);
}
void Robot::setCurrentX(double x)
{
  _x0 = _x;
  _x = x;
}
void Robot :: setCurrentY(double y)
{
  _y0 = _y;
  _y = y;
}
void Robot :: setAngle(double angle) {
  _angle = angle;
}
double Robot :: getCurrentX() {
  return _x;
}
double Robot :: getCurrentY() {
  return _y;
}
double Robot :: getPreviousX() {
  return _x0;
}
double Robot :: getPreviousY() {
  return _y0;
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
