#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void enableReports() {
  myIMU.enableRotationVector(50); //Send data update every 50ms
}
int yaw = 0, firstYaw = -1;
void setup()
{
  Serial.begin(38400);
  Serial.println();
  Wire.begin();
  if (myIMU.begin() == false)
  {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
    while (1);
  }
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms
  while (1)
  {
    if (myIMU.dataAvailable() == true)
    {
      firstYaw = ((myIMU.getYaw()) * 180.0 / PI + 360);
      firstYaw = firstYaw % 360;
    }
    if (firstYaw != -1)
      break;
  }
}

void loop()
{
  if (myIMU.dataAvailable() == true)
  {
    yaw = ((myIMU.getYaw()) * 180.0 / PI + 360); // Convert yaw / heading to degrees
    yaw = ((360 - firstYaw) + yaw) % 360;
    yaw = (yaw + 180) % 360;

    Serial.print(String(firstYaw) + "\t" + String(yaw));
    Serial.println();
  }
}
