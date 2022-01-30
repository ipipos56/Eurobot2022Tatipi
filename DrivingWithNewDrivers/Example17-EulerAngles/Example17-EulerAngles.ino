/*
  Using the BNO080 IMU

  Example : Euler Angles
  By: Paul Clark
  Date: April 28th, 2020

  Based on: Example1-RotationVector
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output the Euler angles: roll, pitch and yaw.
  The yaw (compass heading) is tilt-compensated, which is nice.
  https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  https://github.com/sparkfun/SparkFun_MPU-9250-DMP_Arduino_Library/issues/5#issuecomment-306509440

  It takes about 1ms at 400kHz I2C to read a record from the sensor, but we are polling the sensor continually
  between updates from the sensor. Use the interrupt pin on the BNO080 breakout to avoid polling.

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

#include <Wire.h>

#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

void enableReports() {
  myIMU.enableRotationVector(50); //Send data update every 50ms
}
int yaw = 0, firstYaw = -1;
void setup()
{
  Serial.begin(115200);
  Serial.println();
  Serial.println("BNO080 Read Example");

  Wire.begin();
  if (myIMU.begin() == false)
  {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
    while (1);
  }
  Wire.setClock(400000); //Increase I2C data rate to 400kHz

  myIMU.enableRotationVector(50); //Send data update every 50ms
  //  myIMU.softReset();
  //  if (myIMU.hasReset()) {
  //    Serial.println(" ------------------ BNO085 has reset. ------------------ ");
  //    Serial.print(F(" Reason: "));
  //    Serial.println(myIMU.resetReason());
  //    enableReports(); // We'll need to re-enable reports after any reset.
  //  }
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
