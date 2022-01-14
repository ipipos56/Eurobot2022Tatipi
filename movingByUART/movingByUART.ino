#include <ArduinoJson.h>
#include <EncButton.h>

#define EB_BETTER_ENC
#define pi PI

#define R 0.058/2 * 100
#define _R 0.16 * 100
#define Radius 0.32 * 100
#define speed 255


const int Tick = 460;
const int Tick2 = 599;

int currentX = 0;
int currentY = 0;
int previousX = 0;
int previousY = 0;

bool _finishMoving = 0;

float angleOfRobot = 0;

const float d = 0.1827; //meter
const float r = 0.058 / 2;

const unsigned int IN1 = 25;
const unsigned int IN2 = 24;
const unsigned int EN1 = 3;

const unsigned int IN3 = 23;
const unsigned int IN4 = 22;
const unsigned int EN2 = 2;

const unsigned int IN5 = 27;
const unsigned int IN6 = 26;
const unsigned int EN3 = 4;

float vectorOfRobot[3][2];
float v[2];

bool m1;
bool m2;
bool m3;
EncButton<EB_CALLBACK, 18, 19> enc1;
EncButton<EB_CALLBACK, 16, 17> enc2;
EncButton<EB_CALLBACK, 14, 15> enc3;

void setup()
{
  Serial.begin(115200);
  initialization();
  attachment();

  vectorOfRobot[0][0] = 0;
  vectorOfRobot[0][1] = 1;

  v[0] = rotateVector(v, vectorOfRobot[0][0], vectorOfRobot[0][1], 4 * PI / 3)[0];
  v[1] = rotateVector(v, vectorOfRobot[0][0], vectorOfRobot[0][1], 4 * PI / 3)[1];

  vectorOfRobot[1][0] = v[0];
  vectorOfRobot[1][1] = v[1];

  v[0] = rotateVector(v, vectorOfRobot[0][0], vectorOfRobot[0][1], 2 * PI / 3)[0];
  v[1] = rotateVector(v, vectorOfRobot[0][0], vectorOfRobot[0][1], 2 * PI / 3)[1];

  vectorOfRobot[2][0] = v[0];
  vectorOfRobot[2][1] = v[1];

}
void loop()
{
  delay(1000);
  messageCheck();
  //rotationForHex(PI / 2, 12);
  //Serial.println("RobotOfAngle:\t" + String(angleOfRobot));
    //currentMovement(97, 15);
    //Serial.println("RobotOfAngle:\t" + String(angleOfRobot));
    //currentMovement(41, 78);
    //Serial.println("RobotOfAngle:\t" + String(angleOfRobot));
    //currentMovement(-32, 55);
    //Serial.println("RobotOfAngle:\t" + String(angleOfRobot));
  while (1);
}

void messageCheck()
{
  bool checked = false;
  while(!checked) {
    String data = Serial.readStringUntil('\n');

    char* json = data.c_str();

    DynamicJsonDocument doc(2048);
    deserializeJson(doc, json);
    String statusMes = doc["status"].as<String>();
    String message = doc["message"].as<String>();
    String stopMotors = doc["stop"].as<String>();
    if(statusMes == "1" and stopMotors == "False")
    {
      checked = true;
      Serial.println(message);
      for(int i = 0;i<doc["points"].size();i++)
      {
         currentMovement(doc["points"][i]["x"], doc["points"][i]["y"]); 
      }
    }
    else
      Serial.println("Error in sending: " +  statusMes + " " + stopMotors);
  }
}

bool currentMovement(float x, float y)
{
  currentX = x;
  currentY = y;
  int _dir;
  float _x = x - previousX;
  float _y = y - previousY;
  float phi12, phi23, phi31;
  float _phi12, _phi23, _phi31;
  float _12[2], _23[2], _31[2];
  float _angle;
  phi12 = safe_acos((_x * vectorOfRobot[0][0] + _y * vectorOfRobot[0][1]) / (sqrt(_x * _x + _y  * _y) * sqrt(vectorOfRobot[0][0] * vectorOfRobot[0][0] + vectorOfRobot[0][1] * vectorOfRobot[0][1])));
  phi23 = safe_acos((_x * vectorOfRobot[1][0] + _y * vectorOfRobot[1][1]) / (sqrt(_x * _x + _y  * _y) * sqrt(vectorOfRobot[1][0] * vectorOfRobot[1][0] + vectorOfRobot[1][1] * vectorOfRobot[1][1])));
  phi31 = safe_acos((_x * vectorOfRobot[2][0] + _y * vectorOfRobot[2][1]) / (sqrt(_x * _x + _y  * _y) * sqrt(vectorOfRobot[2][0] * vectorOfRobot[2][0] + vectorOfRobot[2][1] * vectorOfRobot[2][1])));

  phi12 = min(phi12, 2 * PI - phi12);
  phi23 = min(phi23, 2 * PI - phi23);
  phi31 = min(phi31, 2 * PI - phi31);

  _12[0] = rotateVector(_12, vectorOfRobot[0][0], vectorOfRobot[0][1], phi12)[0];
  _12[1] = rotateVector(_12, vectorOfRobot[0][0], vectorOfRobot[0][1], phi12)[1];
  _23[0] = rotateVector(_23, vectorOfRobot[1][0], vectorOfRobot[1][1], phi23)[0];
  _23[1] = rotateVector(_23, vectorOfRobot[1][0], vectorOfRobot[1][1], phi23)[1];
  _31[0] = rotateVector(_31, vectorOfRobot[2][0], vectorOfRobot[2][1], phi31)[0];
  _31[1] = rotateVector(_31, vectorOfRobot[2][0], vectorOfRobot[2][1], phi31)[1];

  _phi12 = safe_acos((_x * _12[0] + _y * _12[1]) / (sqrt(_x * _x + _y  * _y) * sqrt(_12[0] * _12[0] + _12[1] * _12[1])));
  _phi23 = safe_acos((_x * _23[0] + _y * _23[1]) / (sqrt(_x * _x + _y  * _y) * sqrt(_23[0] * _23[0] + _23[1] * _23[1])));
  _phi31 = safe_acos((_x * _31[0] + _y * _31[1]) / (sqrt(_x * _x + _y  * _y) * sqrt(_31[0] * _31[0] + _31[1] * _31[1])));

  if (_phi12 != 0)
  {
    phi12 = -phi12;
  }
  if (_phi23 != 0)
  {
    phi23 = -phi23;
  }
  if (_phi31 != 0)
  {
    phi31 = -phi31;
  }
  if (min(abs(phi12), min(abs(phi23), abs(phi31))) == abs(phi12))
  {
    _dir = 12;
    _angle = phi12;
  }
  else if (min(abs(phi12), min(abs(phi23), abs(phi31))) == abs(phi23))
  {
    _dir = 23;
    _angle = phi23;
  }
  else
  {
    _dir = 31;
    _angle = phi31;
  }
  rotationOfRobotVectors(_angle);
  float _distance = sqrt(_x * _x + _y * _y);
  angleOfRobot += _angle;
  _finishMoving = 0;
  robotRotating(_angle);
  delay(300);
  enc1.counter = 0;
  enc2.counter = 0;
  enc3.counter = 0;
  _finishMoving = 0;
  PidForMoving(_distance, _dir, 1, 0 , 0 , 2, 0 , 0, 0.1);
  previousX = x;
  previousY = y;
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

float *rotateVector(float *xy, float x, float y, float angle)
{
  xy[0] = x * cos(angle) - y * sin(angle);
  xy[1] = x * sin(angle) + y * cos(angle);
  return xy;
}

void rotationOfRobotVectors(float rad)
{
  v[0] = rotateVector(v, vectorOfRobot[0][0], vectorOfRobot[0][1], rad)[0];
  v[1] = rotateVector(v, vectorOfRobot[0][0], vectorOfRobot[0][1], rad)[1];

  vectorOfRobot[0][0] = v[0];
  vectorOfRobot[0][1] = v[1];

  v[0] = rotateVector(v, vectorOfRobot[1][0], vectorOfRobot[1][1], rad)[0];
  v[1] = rotateVector(v, vectorOfRobot[1][0], vectorOfRobot[1][1], rad)[1];

  vectorOfRobot[1][0] = v[0];
  vectorOfRobot[1][1] = v[1];

  v[0] = rotateVector(v, vectorOfRobot[2][0], vectorOfRobot[2][1], rad)[0];
  v[1] = rotateVector(v, vectorOfRobot[2][0], vectorOfRobot[2][1], rad)[1];

  vectorOfRobot[2][0] = v[0];
  vectorOfRobot[2][1] = v[1];
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

bool rotationForHex(float rad, int dir)
{
  rotationOfRobotVectors(rad);
  angleOfRobot += rad;
  bool _dir = 0;
  if (rad < 0)
    _dir = 1;
  enc1.counter = 0;
  enc2.counter = 0;
  enc3.counter = 0;
  int tick1 = 0, tick2 = 0, tick3 = 0;
  _finishMoving = 0;
  while (1)
  {
    enc1.tick();
    enc2.tick();
    enc3.tick();
    if (m1)
    {
      tick1 = abs(enc1.counter);
      m1 = !m1;
    }
    if (m2)
    {
      tick2 = abs(enc2.counter);
      m2 = !m2;
    }
    if (m3)
    {
      tick3 = abs(enc3.counter);
      m3 = !m3;
    }
    if (abs(tick3)  >= (abs(rad * Radius) / (2 * PI * R)*Tick2))
      _finishMoving = 1;
    if (abs(tick2)  >= (abs(rad * Radius) / (2 * PI * R)*Tick2))
      _finishMoving = 1;
    if (abs(tick1)  >= (abs(rad * Radius) / (2 * PI * R)*Tick2))
      _finishMoving = 1;
    if (_dir)
    {
      switch (dir)
      {
        case 12:

          analogWrite(EN3, 255);
          digitalWrite(IN5, LOW);
          digitalWrite(IN6, HIGH);
          break;
        case 23:
          analogWrite(EN1, 255);
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
          break;
        case 31:
          analogWrite(EN2, 255);
          digitalWrite(IN3, LOW);
          digitalWrite(IN4, HIGH);
          break;
      }
    }
    else
    {
      switch (dir)
      {
        case 12:
          analogWrite(EN3, 255);
          digitalWrite(IN5, HIGH);
          digitalWrite(IN6, LOW);
          break;
        case 23:
          analogWrite(EN1, 255);
          digitalWrite(IN1, HIGH);
          digitalWrite(IN2, LOW);
          break;
        case 31:
          analogWrite(EN2, 255);
          digitalWrite(IN3, HIGH);
          digitalWrite(IN4, LOW);
          break;
      }
    }
    if (_finishMoving == 1)
      break;
  }
  motorStop(1);
  motorStop(2);
  motorStop(3);
  return _finishMoving;
}

bool robotRotating(float rad)
{
  if (abs(rad) > abs(rad - 2 * pi))  rad = to2PI(rad - 2 * pi);
  bool _dir = 0;
  if (rad < 0)
    _dir = 1;
  enc1.counter = 0;
  enc2.counter = 0;
  enc3.counter = 0;
  int tick1 = 0, tick2 = 0, tick3 = 0;
  while (_finishMoving == 0)
  {
    enc1.tick();
    enc2.tick();
    enc3.tick();
    if (m1)
    {
      tick1 = enc1.counter;
      m1 = !m1;
    }
    if (m2)
    {
      tick2 = enc2.counter;
      m2 = !m2;
    }
    if (m3)
    {
      tick3 = enc3.counter;
      m3 = !m3;
    }
    if ((abs(tick1) + abs(tick2) + abs(tick3)) / 3 >= abs(rad * _R) / (2 * PI * R)*Tick2)
      _finishMoving = !_finishMoving;
    if (_dir)
    {
      analogWrite(EN1, 200);
      analogWrite(EN2, 200);
      analogWrite(EN3, 200);
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      digitalWrite(IN5, LOW);
      digitalWrite(IN6, HIGH);
    }
    else
    {
      analogWrite(EN1, 200);
      analogWrite(EN2, 200);
      analogWrite(EN3, 200);
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      digitalWrite(IN5, HIGH);
      digitalWrite(IN6, LOW);
    }
  }
  motorStop(1);
  motorStop(2);
  motorStop(3);
  return _finishMoving;
}

bool PidForMoving(float distance, int direction, int dir, int16_t tick1, int16_t tick2, float kp, float ki, float kd, float dt)
{
  float _distance = abs((distance * Tick) / (2 * PI * R));
  int _motor1;
  int _motor1A;
  int _motor1B;
  int _motor2;
  int _motor2A;
  int _motor2B;
  int _tick1 = 0;
  int _tick2 = 0;
  tick1 = 0;
  tick2 = 0;
  //Serial.println(String(enc1.counter) + "\t" + String(enc2.counter) + "\t" + String(enc3.counter));
  delay(1000);
  enc1.counter = 0;
  enc2.counter = 0;
  enc3.counter = 0;
  while (1)
  {
    if (_finishMoving == 1)
    {
      //Serial.println("UZGHE");
      break;
    }
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
          _tick1 = tick1 * cos(PI / 6);
        }
        if (m2)
        {
          m2 = !m2;
          tick2 = abs(enc2.counter);
          _tick2 = tick2 * cos(PI / 6);
        }
        if ((int)((_tick1 + _tick2) / 2) >= _distance)
        {
          _finishMoving = true;
        }
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
  }
  motorStop(1);
  motorStop(2);
  motorStop(3);
  delay(300);
  return _finishMoving;
}
