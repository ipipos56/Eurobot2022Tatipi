#define pi PI
#define R 0.057/2
double a = pi / 4;
double A[3][3];
double B[3];
double x0 = 0;
double y0 = 0;
double x = 0;
double y = 0;
double t = 1;

const int tick = 580;

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
  int sign1;
  int sign2;
  int sign3;
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

  zero.tick1 = zero.u1 * zero.time * tick / (2 * pi * R);
  zero.tick2 = zero.u2 * zero.time * tick / (2 * pi * R);
  zero.tick3 = zero.u3 * zero.time * tick / (2 * pi * R);

  zero.x_previous = x;
  zero.y_previous = y;



  return zero;
}
robot r;
void setup() {
  r.x_previous = 0;
  r.y_previous = 0;
  Serial.begin(9600);
}

double map(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  bool flag = 0;
  for (r.time = 100; r.time >= 0; r.time--)
  {
    r.x_previous = 0;
    r.y_previous = 0;
    r.x = 0;
    r.y = 2;
    r.angle = 0;
    if ((abs(calculate(r).u1 * 100) > 52) || (abs(calculate(r).u2 * 100) > 52) || (abs(calculate(r).u3 * 100) > 52))
    {
      r.time ++;
      flag = 1;
      break;
    }
    Serial.println(String(flag) + " : " + String(map(calculate(r).u1 * 100,0,52,0,255)) + " : " + String(map(calculate(r).u2 * 100,0,52,0,255)) + " : " + String(map(calculate(r).u3 * 100,0,52,0,255)));
  }
  
  if (flag == 1)
  {
    Serial.println(String(r.time) + " : " + String(calculate(r).u1 * 100) + " : " + String(calculate(r).u2 * 100) + " : " + String(calculate(r).u3 * 100));
    delay(1000000000);
  }
  delay(10);
}
