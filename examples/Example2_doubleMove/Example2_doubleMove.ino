#include "SparkFun_Qwiic_Step.h"
QwiicStep motor;

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if button will acknowledge over I2C
  if (motor.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1);
  }
  Serial.println("Motor acknowledged.");

  motor.QSetMaxSpeed(800);
  motor.QSetSpeed(300);
  motor.QSetAcceleration(950);
//  delay(500);
  motor.QMove(400);

  delay(2000);
  motor.QMove(400);

//  delay(2000);
//  motor.QMove(400);
//
//  delay(2000);
//  motor.QMove(400);
  while (1);
}
