/******************************************************************************
  Demonstrates the isReached interrupt. Qwiic Step will pull the Interrupt
  pin low when motor has reached its destination.

  Priyanka Makin @ SparkFun Electronics
  Original Creation Date: January 10, 2020

  SparkFun labored with love to create this code. Feel like supporting open source hardware?
  Buy a board from SparkFun! https://www.sparkfun.com/products/15951

  This code is Lemonadeware; if you see me (or any other SparkFun employee) at the
  local, and you've found our code helpful, please buy us a round!

  Hardware Connections:
  Attach RedBoard to computer using micro-B USB cable.
  Connect Qwiic Step to Red Board using Qwiic connector cable.
  Connect stepper motor to Qwiic Step using latch terminals.
  Connect INT pin on Qwiic Step to pin 2 on RedBoard.
  Connect power supply (8-35V) to barrel jack or latch terminals.
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
  Wire.begin();

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

  //motor.setStepSize(STEPSIZE_QUARTER); //Turns a 200 step motor into 800 steps.

  motor.enableIsReachedInterrupt(); //INT pin will go low when motor reaches destination
  motor.clearInterrupts();          //Clears both isLimited and isReached bits. This clears associated ints.

  motor.move(200); //Will run with default maxSpeed/accel values.

  long startTime = millis();
  while (1)
  {
    if (digitalRead(QS_INT) == LOW)
    {
      Serial.print("We reached the requested position in ");
      Serial.print(millis() - startTime);
      Serial.println("ms");
      break;
    }

    delay(10);

    if (millis() - startTime > 3000) //Timeout after 3s
    {
      Serial.println("Interrupt did not trigger. Do you have the correct interrupt pin wired?");
      break;
    }
  }

  motor.clearIsReached(); //Clear isReached bit. This will clear associated int.
  delay(10);              //Qwiic Step firmware can take a few ms to get to releasing the INT pin

  if (digitalRead(QS_INT) == HIGH)
    Serial.println("INT pin has returned high. Interrupt cleared.");
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
