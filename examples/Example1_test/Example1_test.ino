#include "SparkFun_Qwiic_Step.h"
QwiicStep motor;

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if button will acknowledge over I2C
  if (motor.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1);
  }
  Serial.println("Motor acknowledged.");

//  //Get firmware version
//  Serial.print("The firmware version is: 0x");
//  int version = motor.getFirmwareVersion();
//  Serial.println(version, HEX);
//
//  motor.QSetMaxSpeed(800);
//  motor.QSetSpeed(300);
//  motor.QSetAcceleration(950);
//  motor.QMoveTo(400);
//
//  motor.HalfStepMode();
//  
//  Serial.print("Max speed: ");
//  Serial.println(motor.QGetMaxSpeed());
//  Serial.print("Set speed: ");
//  Serial.println(motor.QGetSpeed());
//  Serial.print("Acceleration: ");
//  Serial.println(motor.QGetAcceleration());

//  motor.stopWhenLimSwitchPressedEnable();
//  motor.powerDownPosReachedEnable();

  motor.enablePositionReachedInterrupt();
  motor.enableLimSwitchPressedInterrupt();
  
//  motor.stopWhenLimSwitchPressedDisable();
//  motor.powerDownPosReachedDisable();
//  motor.disablePositionReachedInterrupt();
//  motor.disableLimSwitchPressedInterrupt();
}

void loop() {

}
