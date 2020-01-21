/******************************************************************************
  Qwiic Step can divide each step into smaller microsteps. This increases precision
  but also affects the other parameters.

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
  Connect power supply (8-35V) to barrel jack or latching terminals.
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

  motor.setModeRunToPosition();

  //Qwiic Step supports full, 1/2, 1/4, 1/8, and 1/16th microstepping
  //motor.setStepSize(STEPSIZE_FULL);
  //motor.setStepSize(STEPSIZE_HALF);
  motor.setStepSize(STEPSIZE_QUARTER); //Turns a 200 step motor into 800 steps.
  //motor.setStepSize(STEPSIZE_EIGHTH);
  //motor.setStepSize(STEPSIZE_SIXTEENTH);

  //The number of steps is related to the step size. If you have a 200 step motor, and your step
  //size is half, it will take 400 steps to complete one turn.
  //motor.move(1 * 200); //Turn one exact rotation of a 200 step stepper motor with full step mode
  //motor.move(2 * 200); //Turn one exact rotation of a 200 step stepper motor with 1/2 step mode
  motor.move(4 * 200); //Turn one exact rotation of a 200 step stepper motor with 1/4 step mode

  //Speed is also related to the step size. If step size is 1/2, that turns a 200 step
  //motor into a 400 step motor. If a speed of 300 was good at full step mode, you'll
  //need to increase your speed accordingly.
  //motor.setSpeed(1 * 300); //Turn at a rate of 300 steps per second
  //motor.setSpeed(2 * 300); //Turn at a rate of 600 steps per second
  motor.setSpeed(4 * 300); //Turn at a rate of 1200 steps per second

  //Your speed can get very large. You may need to increase your max speed.
  //motor.setMaxSpeed(1 * 600);
  //motor.setMaxSpeed(2 * 600);
  motor.setMaxSpeed(4 * 600);

  while (motor.isRunning() == true)
  {
    printStatus();
    delay(50);
  }
  Serial.println("Motor got to position.");
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