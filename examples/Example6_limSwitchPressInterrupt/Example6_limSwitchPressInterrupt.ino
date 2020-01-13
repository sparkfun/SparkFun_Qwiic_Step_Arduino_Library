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
  Attach Red Board to computer using micro-B USB cable.
  Connect Qwiic Step to Red Board using Qwiic connector cable.
  Connect stepper motor to Qwiic Step easy to use using latch terminals.
  Connect power supply (8-35V) to barrel jack or latch terminals.
  Connect a button to Qwiic Step limit switch JST connector.
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

  //check if the motor will acknowledge I2C
  if (motor.begin() == false)
  {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1)
      ;
  }
  Serial.println("Motor acknowledged.");

  printStatus();

  //enable interrupt trigger when limit switch is pressed
  motor.clearIsLimited(); //Clear isLimited bit. This will clear associated int.
  motor.enableIsLimitedInterrupt();

  printStatus();

  Serial.println("Running motor until limit switch is pressed or 8 seconds goes by");

  motor.setAcceleration(5);
  motor.move(1000); //Will run with default maxSpeed/accel values.

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

  //motor.stop(); //Stop all movement including any move or moveTo commands.

  motor.clearIsLimited(); //Clear isLimited bit. This will clear associated int.
  delay(100);
  printStatus();
  if (digitalRead(QS_INT) == HIGH)
    Serial.println("INT pin is high. No interrupts.");
  else
    Serial.println("INT pin is still low. Interrupt failed to clear.");
}

void loop()
{
}

void printStatus()
{
  Serial.print("Qwiic Step Status: ");
  if (motor.isRunning())
    Serial.print(" (isRunning)");
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
