
#include <EncButton.h>

EncButton<EB_CALLBACK, 2, 3> enc;
EncButton<EB_CALLBACK, 4, 5> enc2;
EncButton<EB_CALLBACK, 6, 7> enc3;

void attachment()
{
  enc.attach(TURN_HANDLER, myTurn);
  enc.attach(TURN_H_HANDLER, myTurnH);

  enc.attach(RIGHT_HANDLER, myRight);
  enc.attach(LEFT_HANDLER, myLeft);

  enc.attach(RIGHT_H_HANDLER, myRightH);
  enc.attach(LEFT_H_HANDLER, myLeftH);

  enc2.attach(TURN_HANDLER, myTurn2);
  enc2.attach(TURN_H_HANDLER, myTurnH2);

  enc2.attach(RIGHT_HANDLER, myRight);
  enc2.attach(LEFT_HANDLER, myLeft);

  enc2.attach(RIGHT_H_HANDLER, myRightH);
  enc2.attach(LEFT_H_HANDLER, myLeftH);

  enc3.attach(TURN_HANDLER, myTurn3);
  enc3.attach(TURN_H_HANDLER, myTurnH);

  enc3.attach(RIGHT_HANDLER, myRight);
  enc3.attach(LEFT_HANDLER, myLeft);

  enc3.attach(RIGHT_H_HANDLER, myRightH);
  enc3.attach(LEFT_H_HANDLER, myLeftH);
}

void setup() {
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

void myTurnH() {
  Serial.println("TURN_H_HANDLER");
  Serial.println(enc.getDir()); // направление поворота
}

void myTurnH2() {
  Serial.println("TURN_H_HANDLER");
  Serial.println(enc2.getDir()); // направление поворота
}

void myTurnH3() {
  Serial.println("TURN_H_HANDLER");
  Serial.println(enc3.getDir()); // направление поворота
}

void myRight() {
  Serial.println("RIGHT_HANDLER");
}
void myLeft() {
  Serial.println("LEFT_HANDLER");
}
void myRightH() {
  Serial.println("RIGHT_H_HANDLER");
}
void myLeftH() {
  Serial.println("LEFT_H_HANDLER");
}
void myClick() {
  Serial.println("CLICK_HANDLER");
}
void myHolded() {
  Serial.println("HOLDED_HANDLER");
}
void myStep() {
  Serial.println("STEP_HANDLER");
}

// =============== LOOP =============
void loop() {
  enc.tick();   // обработка всё равно здесь
  //enc2.tick();
  //enc3.tick();
}
