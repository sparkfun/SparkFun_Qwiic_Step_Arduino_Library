/******************************************************************************
  Tests single bit writes to Qwiic Step. You can check that this works if you 
  printState() in the Qwiic Step firmware. 

  Priyanka Makin @ SparkFun Electronics
  Original Creation Date: January 10, 2020
  
  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15951

  This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Hardware Connections:
  Attach Red Board to computer using micro-B USB cable.
  Connect Qwiic Step to Red Board using Qwiic connector cable.
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

  //check if the motor will acknowledge over I2C
  if (motor.begin() == false)
  {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1)
      ;
  }
  Serial.println("Motor acknowledged.");

  motor.enableIsReachedInterrupt();
  //  motor.disableIsReachedInterrupt();

  motor.disableIsLimitedInterrupt();
  //  motor.disableIsLimitedInterrupt();

  motor.enableStopWhenLimSwitchPressed();
  //  motor.disableStopWhenLimSwitchPressed();

  motor.enableDisableMotorWhenPosReached();
  //  motor.disableDisableMotorWhenPosReached();
}

void loop()
{
}