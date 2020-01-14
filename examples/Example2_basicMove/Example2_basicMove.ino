/******************************************************************************
  Move a 200 step stepper motor exactly one rotation.
  Shows how to change the microstep value as well as the speed and acceleration settings.

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

  //check if motor will acknowledge over I2C
  if (motor.begin() == false)
  {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1)
      ;
  }
  Serial.println("Motor acknowledged.");

  //Pick whichever micro-stepping setting you would like
  //motor.fullStepMode();
  //  motor.halfStepMode();
  //  motor.quarterStepMode();
  //  motor.eighthStepMode();
  //  motor.sixteenthStepMode();

  //Set/write all accelstepper parameters
  //We must set a max speed and accel before a move command for 'normal looking' operation
  //motor.setMaxSpeed(500); //There is a limit here. 1000 at full step fails to rotate one full. If maxSpeed is greater than the ability for the mega to service run() then the stepper drops steps.

  motor.move(200); //Turn one exact rotation of a 200 step stepper motor
  //motor.move(16 * 200); //Turn one exact rotation of a 200 step stepper motor with 1/16th step mode
}

void loop()
{
}