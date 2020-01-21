/******************************************************************************
  A configurator for changing the I2C address on Qwiic Step.

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

  //Scan the bus and see if we can detect a Qwiic Step out there...
  byte address = 0;
  for (address = 1; address < 127; address++)
  {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0)
    {
      if (motor.begin(address) == true)
      {
        Serial.print("Qwiic Step detected at address ");
        Serial.print(address);
        Serial.println("!");
        break;
      }
    }
  }
  if (address == 127)
  {
    Serial.println("Qwiic Step was not found on the bus. Are you sure it's attached over Qwiic? Freezing...");
    while (1)
      ;
  }
}

void loop()
{
  Serial.println("Enter a new I2C address for the Qwiic Step to use (8 to 119 is allowed, 82 is default):");

  while (Serial.available() == 0) //Wait for user to enter something
    ;

  Serial.setTimeout(1); //Skip the parseInt waiting, we know we have chacters waiting
  int newAddress = Serial.parseInt();

  if (newAddress > 0x07 && newAddress < 0x78)
  {
    Serial.print("Attempting to set device address to ");
    Serial.println(newAddress);

    if (motor.setI2Caddress(newAddress) == true)
    {
      delay(10); //Wait for hardware to move to new address

      if (motor.isConnected() == true)
      {
        Serial.print("Success! You can now talk to this device with the motor.begin(");
        Serial.print(newAddress);
        Serial.println(") function. Freezing...");
        while (1)
          ;
      }
    }
    Serial.println("Device address set failed!");
  }
  else
  {
    Serial.println("Address out of range! Please choose 8 to 119.");
  }
}
