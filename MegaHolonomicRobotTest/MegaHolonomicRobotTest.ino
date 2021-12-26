#include <EncButton.h>

#define pi PI
#define R 0.057/2

EncButton<EB_CALLBACK, 2, 3> enc;
EncButton<EB_CALLBACK, 4, 5> enc2;
EncButton<EB_CALLBACK, 6, 7> enc3;

double a = pi/4;
double A[3][3];
double B[3];
double x0 = 0;
double y0 = 0;
double x = 0;
double y = 0;
double t = 1;

const int tick = 605;

void attachment()
{
  enc.attach(TURN_HANDLER, myTurn);
  enc2.attach(TURN_HANDLER, myTurn2);
  enc3.attach(TURN_HANDLER, myTurn3);
}

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
  
  A[0][0] = 1/3;
  A[0][1] = -(sqrt(3)*sin(a))/3;
  A[0][2] = (sqrt(3)*cos(a))/3;
  
  A[1][0] = 1/3;
  A[1][1] = (sqrt(3)*sin(a))/6-cos(a)/2;
  A[1][2] = -sin(a)/2 - (sqrt(3)*cos(a))/6;
  
  A[2][0] = 1/3;
  A[2][1] = (sqrt(3)*sin(a))/6+cos(a)/2;
  A[2][2] = sin(a)/2 - (sqrt(3)*cos(a))/6;
  
  B[0] = (double)0;
  B[1] = ((x-x0)*sqrt(3))/(R*t);
  B[2] = ((y-y0)*sqrt(3))/(R*t);
  
  zero.w1 = A[0][1]*B[1]+A[0][2]*B[2];
  zero.w2 = A[1][1]*B[1]+A[1][2]*B[2];
  zero.w3 = A[2][1]*B[1]+A[2][2]*B[2];

  zero.u1 = zero.w1 * R;
  zero.u2 = zero.w2 * R;
  zero.u3 = zero.w3 * R;

  zero.tick1 = zero.u1*tick/(2*pi*R);
  zero.tick2 = zero.u2*tick/(2*pi*R);
  zero.tick3 = zero.u3*tick/(2*pi*R);
  
  zero.x_previous = x;
  zero.y_previous = y;


  
  return zero;
}
robot r;
void setup() {
  r.x_previous = 0;
  r.y_previous = 0;
  Serial.begin(115200);
  attachment(); 
}

void myTurn() {
  Serial.println("1:" + String(enc.counter));  // вывести счётчик
}

void myTurn2() {
  Serial.println("2:" + String(enc2.counter));  // вывести счётчик
}
void myTurn3() {
  Serial.println("3:" + String(enc3.counter));  // вывести счётчик
}

void loop() {
r.x = 0.3;
r.y = 0.2;
r.angle = pi/4;
r.time = 10;
Serial.println(calculate(r).tick3);
delay(50000);
}
