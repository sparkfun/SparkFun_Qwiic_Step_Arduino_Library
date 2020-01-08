//Describe hardware setup here!

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

  //enable interrupt trigger when limit switch is pressed
  motor.enableLimSwitchPressedInterrupt();

  //assume that someone pressed the limit switch at this point

  //let 5 seconds pass by
  delay(5000);

  //clear the limit switch press flag
  Serial.println("Hello, I'm clearing the interrupt flag");
  motor.clearLimSwitchPressInterrupt();
}

void loop(){
  
}
