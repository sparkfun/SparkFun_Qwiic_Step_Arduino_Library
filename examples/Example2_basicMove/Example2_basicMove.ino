#include "SparkFun_Qwiic_Step.h"
QwiicStep motor;

void setup(){
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if motor will acknowledge over I2C
  if (motor.begin() == false){
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("Motor acknowledged.");

  motor.fullStepMode();
//  motor.halfStepMode();
//  motor.quarterStepMode();
//  motor.eighthStepMode();
//  motor.sixteenthStepMode();

  //Set/write all accelstepper parameters
  motor.QSetMaxSpeed(800);
  motor.QSetSpeed(300);
  motor.QSetAcceleration(950);
  motor.QMove(400);

  //Get/read the previously set accelstepper parameters
  Serial.print("Max speed: ");
  Serial.println(motor.QGetMaxSpeed());
  Serial.print("Set speed: ");
  Serial.println(motor.QGetSpeed());
  Serial.print("Acceleration: ");
  Serial.println(motor.QGetAcceleration());
}

void loop(){
  
}
