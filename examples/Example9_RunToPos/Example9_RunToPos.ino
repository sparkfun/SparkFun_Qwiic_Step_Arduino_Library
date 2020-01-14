/******************************************************************************
  Given a position, run at speed (no accel/decel) to that position then stop.
  
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

  //Run without accelerations to a position
  motor.setMaxSpeed(1000);

  motor.move(200); //This move must come before setSpeed.

  //Speeds are in steps per second.
  //Positive is clockwise, negative is counter clockwise.
  //Speeds of more than 1000 are unreliable.
  //Decimal values are allowed. 0.1 = 1 step every ten seconds.
  motor.setSpeed(350);
  motor.modeRunSpeedToPosition(); //Tell the motor to run at that speed until we have arrived at position
}

void loop()
{
}