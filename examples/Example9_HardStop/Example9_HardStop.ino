/******************************************************************************
  Demonstrates stopping the motor instantaneously, no deceleration.

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

#include "SparkFun_Qwiic_Step.h" //Click here to get the library: http://librarymanager/All#Qwiic_Step by SparkFun
QwiicStep motor;

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if the motor will acknowledge I2C
  if (motor.begin() == false)
  {
    Serial.println("Device did not acknowledge! Freezing.");
  }
  Serial.println("Motor acknowledged.");

  motor.setMaxSpeed(1000);
  motor.setSpeed(-350);
  motor.modeRunSpeed();

  delay(2000);

  //Both stop methods cause an immediate stop when in runSpeed mode.
  //motor.stop(); //Stops motor and overwrites speed to 0
  motor.hardStop(); //Causes accelStepper library to stop running runSpeed()
}

void loop()
{
}