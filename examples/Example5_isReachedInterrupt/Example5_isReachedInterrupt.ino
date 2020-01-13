/******************************************************************************
  Tests the setting and clearing of the positionReached interrupt.

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

  motor.clearIsReached(); //Clear isReached bit. This will clear associated int.
  motor.enableIsReachedInterrupt();

  //move motor
  motor.QSetMaxSpeed(800);
  motor.QSetSpeed(300);
  motor.QSetAcceleration(950);
  motor.QMove(400);

  long startTime = millis();
  while (1)
  {
    if (digitalRead(QS_INT) == LOW)
    {
      Serial.print("Reached requested position in ");
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
  if (digitalRead(QS_INT) == HIGH)
    Serial.println("INT pin has returned high. Interrupt cleared.");
  else
    Serial.println("INT pin is still low. Interrupt failed to clear.");
}

void loop()
{
}