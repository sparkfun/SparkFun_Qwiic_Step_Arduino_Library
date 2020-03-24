/******************************************************************************
  Qwiic Step will do an emergency stop if the EStop pin is pulled low.
  The motor coils will continue to be energized but the head will not move
  until the user clears the eStopped bit.

  An EStop causes all settings to be cleared to zero.

  Optionally, you can disable the motor after EStop. This will cause the motor
  to de-energize allowing the head to spin freely.

  Multiple Qwiic Step EStop pins can be connected together to one big 'Uh Oh' stop button.

  Priyanka Makin @ SparkFun Electronics
  Original Creation Date: January 10, 2020

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15951

  This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Hardware Connections:
  Attach RedBoard to computer using a USB cable.
  Connect Qwiic Step to Red Board using Qwiic connector cable.
  Connect stepper motor to Qwiic Step using latch terminals.
  Connect a button to Qwiic Step EStop connector: https://www.sparkfun.com/products/8671
  Connect power supply (8-35V) to barrel jack or latch terminals.
  Attach a button with JST to E-Stop connector on Qwiic Step.

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

  motor.setModeRunContinuous();
  motor.setSpeed(200);

  motor.clearEStop(); //Clear any previous EStop situation
  motor.enable();     //Energize the coils if needed

  motor.enableDisableMotorOnEStop(); //Optional - will cause motor to spin freely after an EStop
  //motor.disableDisableMotorOnEStop(); //Optional - will cause motor to hold after and EStop (default)

  Serial.println("Press 'g' to go. Press 'c' to clear the EStop condition.");
}

void loop()
{
  printStatus();

  if (Serial.available())
  {
    byte incoming = Serial.read();

    if (incoming == 'c')
    {
      motor.clearEStop();
    }
    else if (incoming == 'g')
    {
      motor.enable();         //Re-energize coils if needed
      motor.setMaxSpeed(800); //4 full rotations per second on a 200 step motor
      motor.setSpeed(200);
    }
  }

  delay(100);
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
  {
    Serial.print(" (isEStopped)");
    Serial.print(" - Press 'c' to clear the EStop interrupt condition");
  }
  Serial.println();
}
