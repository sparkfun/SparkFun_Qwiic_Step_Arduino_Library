/******************************************************************************
  Tests motor movement functions. We can get and set the current turning values.

  Priyanka Makin @ SparkFun Electronics
  Original Creation Date: January 10, 2020

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15951

  This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Hardware Connections:
  Attach Red Board to computer using micro-B USB cable.
  Connect Qwiic Step to Red Board using Qwiic connector cable.
  Connect stepper motor to Qwiic Step easy to use using latch terminals.
  Connect power supply (8-35V) to barrel jack or latch terminals.
  Open Serial Monitor at 115200 baud.

  Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_Qwiic_Step.h"  //Click here to get the library: http://librarymanager/All#Qwiic_Step by SparkFun
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

  //Pick whichever micro-stepping setting you would like
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
