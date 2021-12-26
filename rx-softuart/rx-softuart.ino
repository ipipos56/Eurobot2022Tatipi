// принимаем данные от gbus mini

// подключаем софт юарт
#include "softUART.h"
softUART<3> UART(10000); // пин 4, скорость 1000
// подключаем GBUS
#include "GBUS.h"
GBUS bus(&UART, 3, 20); // обработчик UART, адрес 3, буфер 20 байт

// структура для приёма
struct myStruct {
  String data;
};

void setup() {
  Serial.begin(115200); // сериал для отладки (вывод в монитор)
}

void loop() {
  // в тике сидит отправка и приём
  bus.tick();
  if (bus.gotData()) {
    myStruct rxData;
    bus.readData(rxData);
    Serial.println(rxData.data);
    Serial.println();
  }
}
