#include <EncButton.h>

#define EB_BETTER_ENC
#define pi PI

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
    Robot(double x0, double y0 , double x, double y, double angle)
    {
      this -> x0 = x0;
      this -> y0 = y0;
      this -> x = x;
      this -> y = y;
    }
    void setCurrentX(double x);
    void setCurrentY(double y);
    void setAngle(double angle);
    double getCurrentX();
    double getCurrentY();
    double getPreviousX();
    double getPreviousY();
    void setA()
    {
      this -> A[0][0] = -d;
      this -> A[0][1] = 1;
      this -> A[0][2] = 0;
      this -> A[1][0] = -d;
      this -> A[1][1] = -1 / 2;
      this -> A[1][2] = -sin(PI / 3);
      this -> A[2][0] = -d;
      this -> A[2][1] = -1 / 2;
      this -> A[2][2] = sin(PI / 3);
    }
    void attachment();
    void initialization();
  private:
    double x0;
    double y0;
    double x;
    double y;
    double angle;
    double A[3][3];
};
Robot firstRobot();
void setup()
{
  Serial.begin(115200);
  firstRobot.initialization();
  firstRobot.attachment();
}
void loop()
{
}
void Robot::initialization()
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
void Robot::attachment()
{
  enc1.attach(TURN_HANDLER, myTurn);
  enc2.attach(TURN_HANDLER, myTurn2);
  enc3.attach(TURN_HANDLER, myTurn3);
}
void Robot::setCurrentX(double x)
{
  this -> x0 = this -> x;
  this -> x = x;
}
void Robot :: setCurrentY(double y)
{
  this -> y0 = this -> y;
  this -> y = y;
}
void Robot :: setAngle(double angle) {
  this ->angle = angle;
}
double Robot :: getCurrentX() {
  return this -> x;
}
double Robot :: getCurrentY() {
  return this -> y;
}
double Robot :: getPreviousX() {
  return this -> x0;
}
double Robot :: getPreviousY() {
  return this -> y0;
}
void myTurn()
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
