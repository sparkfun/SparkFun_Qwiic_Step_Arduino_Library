#include "SparkFun_Qwiic_Step.h"
QwiicStep motor;

void setup(){
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if the motor will acknowledge over I2C
  if (motor.begin() == false){
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("Motor acknowledged.");

  motor.enablePositionReachedInterrupt(); //set bit 0 to 1. register should become 0x01.
//  motor.disablePositionReachedInterrupt();
  
  motor.enableLimSwitchPressedInterrupt(); //set bit 2 to 1. regiter should beceom 0x05
//  motor.disableLimSwitchPressedInterrupt();

  motor.enableStopWhenLimSwitchPressed();
//  motor.disableStopWhenLimSwitchPressed();

  motor.enableDisableMotorWhenPosReached();
//  motor.disableDisableMotorWhenPosReached();
}

void loop(){
  
}
