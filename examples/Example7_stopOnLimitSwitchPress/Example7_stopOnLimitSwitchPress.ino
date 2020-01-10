/******************************************************************************
  Tests stopping the motor on limit switch press.

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
  Connect a button to Qwiic Step limit switch JST connector.
  Open Serial Monitor at 115200 baud.

  Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_Qwiic_Step.h"  //Click here to get the library: http://librarymanager/All#Qwiic_Step by SparkFun 
QwiicStep motor;

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if the motor will acknowledge I2C
  if (motor.begin() == false){
    Serial.println("Device did not acknowledge! Freezing.");
    while(1);
  }
  Serial.println("Motor acknowledged.");

  //Setting this bit tells the slave that we want to stop the motor
  //when the limit switch is pressed
  motor.enableStopWhenLimSwitchPressed();
  
  //Run the motor
  motor.QSetMaxSpeed(800);
  motor.QSetAcceleration(950);
  //In full step mode, this should spin my motor 4 times
  //My stepper motor has 400 steps
  motor.QMove(1600);

  //At some point, press the limit switch
  Serial.println("Please press the limit switch");
  //Motor should stop turning

  //Wait 10 seconds and start turning again
  delay(10000);
  motor.QMove(400);
}

void loop() {

}
