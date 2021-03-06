/******************************************************************************
  Move to a location, then once the position is reached, turn the opposite direction.
  We use the isReached() status bit to see if the motor has reached our intended destination.

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

  motor.move(200); //Turn one exact rotation of a 200 step stepper motor
  //motor.move(16 * 200); //Turn one exact rotation of a 200 step stepper motor with 1/16th step mode

  while (motor.isReached() == false)
  {
    printStatus();
    delay(10);
  }
  Serial.println("Motor arrived to position. Now run motor in opposite direction.");

  motor.move(-200); //Turn in opposite direction
}

void loop()
{
}

void printStatus()
{
  Serial.print("Qwiic Step Status: ");
  if (motor.isRunning())
    Serial.print(" (isRunning)");
  else
    Serial.print(" (Stopped)");
  if (motor.isAccelerating())
    Serial.print(" (isAccelerating)");
  if (motor.isDecelerating())
    Serial.print(" (isDecelerating)");
  if (motor.isReached())
    Serial.print(" (isReached)");
  if (motor.isLimited())
    Serial.print(" (isLimited)");
  if (motor.isEStopped())
    Serial.print(" (isEStopped)");
  Serial.println();
}