
#include <EncButton.h>

EncButton<EB_CALLBACK, 2, 3> enc;
EncButton<EB_CALLBACK, 4, 5> enc2;
EncButton<EB_CALLBACK, 6, 7> enc3;

void attachment()
{
  enc.attach(TURN_HANDLER, myTurn);
  enc2.attach(TURN_HANDLER, myTurn2);
  enc3.attach(TURN_HANDLER, myTurn3);
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

// =============== LOOP =============
void loop() {
  enc.tick();   // обработка всё равно здесь
  enc2.tick();
  enc3.tick();
}
