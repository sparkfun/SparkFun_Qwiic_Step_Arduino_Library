/******************************************************************************
  Demonstrates the isLimited interrupt. The motor will run until an interrupt occurs
  indicating the limit switch on Qwiic Step has been pressed.

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
  Connect INT pin on Qwiic Step to pin 2 on RedBoard.
  Connect a button to Qwiic Step limit switch JST connector: https://www.sparkfun.com/products/8671
  Open Serial Monitor at 115200 baud.

  Distributed as-is; no warranty is given.
******************************************************************************/

#include "SparkFun_Qwiic_Step.h" //Click here to get the library: http://librarymanager/All#Qwiic_Step by SparkFun
QwiicStep motor;

#define QS_INT 2 //Any GPIO will work. Connect pin 2 on your board to the INT pin on Qwiic Step

void setup()
{
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  pinMode(QS_INT, INPUT_PULLUP);

  //Check if Qwiic Step is correctly connected to I2C
  if (motor.begin() == false)
  {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1)
      ;
  }
  Serial.println("Motor acknowledged.");

  printStatus();

  motor.setStepSize(STEPSIZE_QUARTER); //Turns a 200 step motor into 800 steps.

  motor.enableIsLimitedInterrupt();         //INT pin will go low if limit switch is closed
  motor.enableStopWhenLimitSwitchPressed(); //Stop motor when limit switch is close
  motor.clearInterrupts();                  //Clears both isLimited and isReached bits. This clears associated ints.

  printStatus();

  Serial.println("Running motor until limit switch is pressed or 8 seconds goes by");

  motor.move(6000); //Will run with default maxSpeed/accel values.

  long startTime = millis();
  while (1)
  {
    printStatus();
    if (digitalRead(QS_INT) == LOW)
    {
      Serial.print("Limit switch pressed after ");
      Serial.print(millis() - startTime);
      Serial.println("ms");
      break;
    }

    //Polling method
    if (motor.isReached() == true)
    {
      Serial.println("Motor reached requested position.");
      break;
    }

    delay(10);

    if (millis() - startTime > 8000) //Timeout after 8s
    {
      Serial.println("Motor failed to get to position in time and interrupt didn't trigger. Do you have the correct interrupt pin wired?");
      break;
    }
  }

  printStatus();

  motor.stop(); //Move to 0. This will cause motor to spin to a stop. isReached bit will be set and interrupt will fire if enabled.

  //Wait for motor to spin to a stop
  while (motor.isRunning() == true)
  {
    delay(50);
  }

  motor.clearInterrupts(); //Clears both isReached and isLimited bits
  delay(10);               //Qwiic Step firmware can take a few ms to get to releasing the INT pin

  if (digitalRead(QS_INT) == HIGH)
    Serial.println("INT pin is high. No interrupts.");
  else
    Serial.println("INT pin is still low. Interrupt failed to clear.");

  printStatus();
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