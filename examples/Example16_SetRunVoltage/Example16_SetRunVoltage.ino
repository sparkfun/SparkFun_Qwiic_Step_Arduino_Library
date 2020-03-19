/******************************************************************************
  Qwiic Step defaults to a hold and run voltage of 1.2V. This is about 1A per
  phase but varies on the motor and step configuration you have.
  The A4988 IC is capable of 2A total (1A per phase) but requires heat sinking
  and a larger power supply.

  In addition, Qwiic Step can be configured to have seperate hold and run current
  maximums. For example, while holding the max voltage can be set to 0.1V where
  as the run voltage can be set to 2V.

  This is achieved on Qwiic Step by changing the voltage going into the
  CURRENT_REFERENCE pin on the A4988.

  This example shows how to modify the run and hold voltages.

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

  motor.setHoldVoltage(0.050); //Set hold voltage to 0.05V
  motor.setRunVoltage(2.000);  //Set run voltage to 2V

  motor.move(200); //Turn one exact rotation of a 200 step stepper motor

  //Motor will be moving...
  //It should be easy to turn the motor once it has finished moving.
}

void loop()
{
}