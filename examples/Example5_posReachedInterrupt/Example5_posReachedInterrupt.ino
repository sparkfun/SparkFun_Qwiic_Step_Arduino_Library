#include "SparkFun_Qwiic_Step.h"
QwiicStep motor;

void setup(){
  Serial.begin(115200);
  Serial.println("Qwiic step examples");
  Wire.begin(); //Join I2C bus

  //check if the motor will acknowledge I2C
  if (motor.begin() == false){
    Serial.println("Device did not acknowledge! Freezing.");
    while(1); 
  }
  Serial.println("Motor acknowledged.");

  motor.enablePositionReachedInterrupt();

  //move motor
  motor.QSetMaxSpeed(800);
  motor.QSetSpeed(300);
  motor.QSetAcceleration(950);
  motor.QMove(400);

  //wait 3 seconds
  //DEBUG: should see "Position reached!" and "Hello" on slave serial monitor
  delay(3000);

  //clear position reached interrupt flag
  //motor status should still be "isReached" but interrupt pin should go high
  //DEBUG: should only see "Position reached!" on slave serial monitor
  Serial.println("Hello, I'm clearing the interrupt flag");
  motor.clearIsReachedInterrupt();
} 

void loop(){
  
}
