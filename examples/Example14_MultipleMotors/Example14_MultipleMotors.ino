/******************************************************************************
  The Qwiic Step library allows for some advanced I2C configurations including:

  * 400kHz I2C communication
  * Talk to multiple Qwiic steps by passing the I2C address in motor.begin(deviceAddress);
  * Change the Wire port by passing it as the second argument in motor.begin(deviceAddress, WirePort);

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
  Attach a second Qwiic Step board and motor using Qwiic connector cable to Qwic Step 1.
  Open Serial Monitor at 115200 baud.

  Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_Qwiic_Step.h" //Click here to get the library: http://librarymanager/All#Qwiic_Step by SparkFun
QwiicStep motor1;
QwiicStep motor2;

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin();
  Wire.setClock(400000); //Fast 400kHz I2C

  //Connect multiple Qwiic Steps
  if (motor1.begin() == false)
  {
    Serial.println("Qwiic Step at default I2C address did not connect");
    while (1)
      ;
  }
  if (motor2.begin(25) == false)
  {
    Serial.println("Qwiic Step at address 25 did not connect");
    while (1)
      ;
  }

  motor1.move(200);  //Turn one exact rotation of a 200 step stepper motor
  motor2.move(-400); //Turn two exact rotations CCW
}

void loop()
{
}
