/******************************************************************************
  Qwiic Step defaults to a hold and run current of 1A total (500mA per phase).
  The A4988 IC is capable of 2A total (1A per phase) but requires heat sinking
  and a larger power supply.

  In addition, Qwiic Step can be configured to have seperate hold and run current
  maximums. For example, while holding the max current can be set to 100mA where
  as the run current can be set to 800mA.

  This is achieved on Qwiic Step by changing the voltage going into the
  CURRENT_REFERENCE pin on the A4988.

  This example shows how to modify the run and hold max current values.

  Priyanka Makin @ SparkFun Electronics
  Original Creation Date: January 10, 2020

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15951

  This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Hardware Connections:
  Attach RedBoard to computer using a USB cable.
  Connect Qwiic Step to Red Board using Qwiic cable.
  Connect stepper motor to Qwiic Step using latching terminals.
  Connect power supply (8-35V) to barrel jack or using latching terminals.
  Open Serial Monitor at 115200 baud.

  Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_Qwiic_Step.h" //Click here to get the library: http://librarymanager/All#Qwiic_Step by SparkFun
QwiicStep motor;

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin();

  //Check if Qwiic Step is correctly connected to I2C
  if (motor.begin() == false)
  {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1)
      ;
  }
  Serial.println("Motor acknowledged.");

  motor.setHoldCurrent(100); //Set hold current to 100mA
  motor.setRunCurrent(800);  //Set run current to 800mA

  motor.move(200); //Turn one exact rotation of a 200 step stepper motor

  //Motor will be moving...
}

void loop()
{
}