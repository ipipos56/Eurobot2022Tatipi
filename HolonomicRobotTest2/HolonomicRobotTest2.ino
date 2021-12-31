const double d = 0.1827; //meter
const double r = 0.058;
  
class Robot
{
  public:
    Robot(double x0, double y0 ,double x, double y, double angle)
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
      this -> A[1][1] = -1/2;
      this -> A[1][2] = -sin(PI/3);
      this -> A[2][0] = -d;
      this -> A[2][1] = -1/2;
      this -> A[2][2] = sin(PI/3);
    }
  private:
    double x0;
    double y0;
    double x;
    double y;
    double angle;
    double A[3][3];
};
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
void Robot :: setAngle(double angle){this ->angle = angle;}
double Robot :: getCurrentX(){return this -> x;}
double Robot :: getCurrentY(){return this -> y;}
double Robot :: getPreviousX(){return this -> x0;}
double Robot :: getPreviousY(){return this -> y0;}
void setup() 
{
}
void loop() 
{
}
