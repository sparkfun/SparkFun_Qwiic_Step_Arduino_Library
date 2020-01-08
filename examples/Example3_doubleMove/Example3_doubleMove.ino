#include "SparkFun_Qwiic_Step.h"
QwiicStep motor;

void setup() {
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if motor will acknowledge over I2C
  if (motor.begin() == false) {
    Serial.println("Device did not acknowledge! Freezing.");
    while (1);
  }
  Serial.println("Motor acknowledged.");
  
  motor.QSetMaxSpeed(800);
  motor.QSetSpeed(300);
  motor.QSetAcceleration(950);
  //Move once
  motor.QMove(400);

  //Wait
  delay(2000);
  //Move a second time
  motor.QMove(400);
}

void loop(){
  
}
