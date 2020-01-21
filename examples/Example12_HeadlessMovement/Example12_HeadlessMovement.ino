/******************************************************************************
  Qwiic Step can be configured to move at power on. This allow for headless
  or 'controllerless' operation.

  To perform a headless continuous move at power-on:
      The mode must be set to runContinuous (will automatically be stored in NVM)
      A speed value needs to be set
      The unlockSpeedNVM register needs to be set to 0xC4 (will cause Speed to be stored in NVM)
  Now and at the next power on, Qwiic Step will run continuous with the speed value even
  if a microcontroller is not connected to Qwiic Step.

  Because this can be dangerous (unexpected movement at power up is bad) the
  move and speed registers are protected by an unlock register. When the unlockMoveNVM
  register is written to 0x59 the contents of the move register are stored into NVM. When
  the unlockSpeedNVM register is written to 0xC4 the contents of the speed register is
  written to NVM. All other NVM enabled registers are automatically tracked in NVM.

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

  motor.enableIsReachedInterrupt(); //Even headless, you can still rely on the interrupt pin on Qwiic Step to be available

  //At power up, run continuously CCW at 350 steps per second
  // motor.setModeRunContinuous(); //Tell the motor to run at the given speed... forever
  // motor.setSpeed(-350);
  // motor.recordSpeedToNVM();

  //At power up, run to a position (600 CW) at a speed of 200 steps per second
  // motor.setModeRunToPosition(); //Tell the motor to move a certain amount
  // motor.move(600);           //Move 600 steps and stop
  // motor.recordMoveToNVM();
  // motor.setSpeed(200);
  // motor.recordSpeedToNVM();

  //At power up, run to a position (-1000 CCW) with acceleration
  // at a speed of 200 steps per second
  motor.setModeRunWithAcceleration(); //Tell the motor to move a certain amount
  motor.setMaxSpeed(600);
  motor.setAcceleration(400);
  motor.move(-2000); //Move 2000 CCW and stop
  motor.recordMoveToNVM();
  //In RunWithAcceleration mode you do not set a speed, the library caclulates it as it accelerates.

  //At power up, disable all movement
  // motor.move(0); //Move 600 steps and stop
  // motor.recordMoveToNVM();
  // motor.setSpeed(0);
  // motor.recordSpeedToNVM();
}

void loop()
{
}