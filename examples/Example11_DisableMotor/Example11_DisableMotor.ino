/******************************************************************************
  You can tell Qwiic Step to disable the motor. This will power down the coils
  saving power and allowing the stepper motor to cool but it will allow the head
  of the stepper motor to run free.

  The .disable() command is immediate and can be used to stop a move but if
  the motor has intertia the disable() command does not guarantee the head will
  stop moving. See the Stop example for that method.

  Priyanka Makin @ SparkFun Electronics
  Original Creation Date: January 10, 2020

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15951

  This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Hardware Connections:
  Attach Red Board to computer using a USB cable.
  Connect Qwiic Step to Red Board using Qwiic connector cable.
  Connect stepper motor to Qwiic Step using latch terminals.
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

  while (motor.isReached() == false)
  {
    printStatus();
    delay(10);
  }
  Serial.println("Motor arrived to position. Disabling the motor.");

  motor.disable();

  Serial.println("Motor head should now turn freely.");

  //motor.enable(); //Will re-energize the coils
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