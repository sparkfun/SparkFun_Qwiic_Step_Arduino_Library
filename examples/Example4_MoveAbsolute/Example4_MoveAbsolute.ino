/******************************************************************************
  Move to an absolute position. This is different from a relative position.

  Relative move: Go 400 steps. Go another -400 steps.
  Absolute move: Go to location 400. Go to location 0.

  The two methods have an equivalent outcome (the motor will move then return)
  but they each have pros/cons and various applications.

  Absolute move is used in conjunction with the setCurrentPosition() to adjust your zero
  point as necessary.

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

  motor.moveTo(100); //Turn 1/2 rotation of a 200 step stepper motor
  while (motor.isRunning() == true)
    delay(100);

  Serial.println("Motor got to position. Let's move to back to our starting position.");

  motor.moveTo(0); //Will move back to original starting position
  while (motor.isRunning() == true)
    delay(100);

  delay(500);
  Serial.println("Now we move to 90 degrees or position 50.");

  motor.moveTo(50);
  while (motor.isRunning() == true)
    delay(100);

  Serial.println("Now let's call this 'home' and move -50 steps to return to our starting position.");

  motor.setPosition(0); //Mark this position as '0'. Any value is accepted.
  motor.moveTo(-50);    //Move to where we started
  while (motor.isRunning() == true)
    delay(100);
}

void loop()
{
}
