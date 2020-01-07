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

//  motor.enablePositionReachedInterrupt(); //set bit 0 to 1. register should become 0x01.

//  motor.fullStepMode();
  
  motor.QSetMaxSpeed(800);
  motor.QSetSpeed(300);
  motor.QSetAcceleration(950);
  
  motor.QMove(400);

  delay(4000);

  motor.QMove(400);
  while(1);
  

  //wait 3 seconds
  //should see "Position reached!" and "Hello" on slave serial monitor
  delay(3000);
  
  //clear position reached interrupt flag
  //position should still be "isReached" but interrupt pin should go high
  //should only see "Position reached!" on slave serial monitor
  Serial.println("Hello, I'm clearing interrupt flag");
  motor.clearIsReachedInterrupt();

//  Serial.print("Max speed: ");
//  Serial.println(motor.QGetMaxSpeed());
//  Serial.print("Set speed: ");
//  Serial.println(motor.QGetSpeed());
//  Serial.print("Acceleration: ");
//  Serial.println(motor.QGetAcceleration());

//  motor.enablePositionReachedInterrupt(); //set bit 0 to 1. register should become 0x01.
//  motor.disablePositionReachedInterrupt();
  
//  motor.enableLimSwitchPressedInterrupt(); //set bit 1 to 1. regiter should beceom 0x03
//  motor.disableLimSwitchPressedInterrupt();

//  motor.enableStopWhenLimSwitchPressed();
//  motor.disableStopWhenLimSwitchPressed();

//  motor.enableDisableMotorWhenPosReached();
//  motor.disableDisableMotorWhenPosReached();
}

void loop() {

}
