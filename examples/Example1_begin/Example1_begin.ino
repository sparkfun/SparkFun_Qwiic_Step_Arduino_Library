#include "SparkFun_Qwiic_Step.h"
QwiicStep motor;

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if motor will acknowledge over I2C
  if (motor.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1);
  }
  Serial.println("Motor acknowledged.");

  //Get firmware version
  Serial.print("The firmware version is: 0x");
  int version = motor.getFirmwareVersion();
  Serial.println(version, HEX);
}

void loop() {

}
