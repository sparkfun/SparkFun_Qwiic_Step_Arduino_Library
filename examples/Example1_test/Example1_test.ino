#include "SparkFun_Qwiic_Step.h"
QwiicStep motor;

void setup(){
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if button will acknowledge over I2C
  if (motor.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("Motor acknowledged.");

  //Get firmware version
  Serial.print("The firmware version is: 0x");
  int version = motor.getFirmwareVersion();
  Serial.println(version, HEX);

  //test writeQuadRegister
//  motor.QSetMaxSpeed(0xaaaaaaaa);
//  motor.QSetMaxSpeed(0x78563412);
//  motor.QMoveTo(0xaaaaaaaa);

    motor.QMoveTo(400);
}

void loop(){
  
}
