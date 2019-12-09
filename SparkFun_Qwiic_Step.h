#ifndef __SparkFun_Qwiic_Step_H__
#define __SparkFun_Qwiic_Step_H__

#include <AccelStepper.h> //DEBUG: maybe.. idk yet
#include <Wire.h>
#include <Arduino.h>
#include "registers.h"

#define DEFAULT_ADDRESS 0x52 //Default I2C address of the motor
#define DEV_ID 0x60          //Device ID of the Qwiic Step

class QwiicStep
{
private:
    TwoWire *_i2cPort;      //Generic connection to user's chosen I2C port
    uint8_t _deviceAddress; //I2C address of the motor

public:
    //Device status
    bool begin(uint8_t address = DEFAULT_ADDRESS, TwoWire &wirePort = Wire); //Sets device I2C address to a user-specified address, over whatever port the user defines.
    bool isConnected();                                                      //Returns true if the button/switch will acknowledge over I2C and false otherwise.
    uint8_t deviceID();                                                      //Returns the 8-bit device ID of the attached device.
    bool checkDeviceID();                                                    //Returns true if the deviceID matches that of the default for the motor.
    uint16_t getFirmwareVersion();                                           //Returns the firmware version of the attched device as a 16-bit integer. The leftmost (high) byte is the major revision number and the rightmost (low) byte is the minor revision number.
    bool setI2Caddress(uint8_t address);                                     //Configures the attach to the I2C bus using the specified address
    uint8_t getI2Caddress();                                                 //Returns the I2C address of the device

    //Motor control
    bool QSetMaxSpeed(float speed);            //stepper.setMaxSpeed()
    bool QSetSpeed(float speed);               //stepper.setSpeed()
    bool QSetAcceleration(float acceleration); //stepper.setAcceleration()
    bool QMoveTo(long absolute);               //stepper.moveTo() -- should have a stepper.move()
    bool QSetStepMode(uint8_t mode);

    //Internal I2C Abstraction
    uint8_t readSingleRegister(Qwiic_Step_Register reg);              //Reads a single 8-bit register.
    uint16_t readDoubleRegister(Qwiic_Step_Register reg);             //Reads a 16-bit register (little endian).
    unsigned long readQuadRegister(Qwiic_Step_Register reg);          //Reads a 32-bit register (little endian).
    bool writeSingleRegister(Qwiic_Step_Register reg, uint8_t data);  //Attempts to write data into a single 8-bit register. Does not check to make sure if it was written successfully. Returns 0 if there wasn't an error on I2C transmission, and 1 otherwise.
    bool writeDoubleRegister(Qwiic_Step_Register reg, uint16_t data); //Attempts to write data into a double (two 8-bit) register. Does not check to make sure if it was written successfully. Returns 0 if there wasn't an error on I2C transmission, and 1 otherwise.
    bool writeQuadRegister(Qwiic_Step_Register reg, long data);
    uint8_t writeSingleRegisterWithReadback(Qwiic_Step_Register reg, uint8_t data);   //Writes data into a single 8-bit register and checks to make sure that the data was written successfully. Returns 0 on no error, 1 on I2C write fail, and 2 if the register doesn't read back the same value that was written.
    uint16_t writeDoubleRegisterWithReadback(Qwiic_Step_Register reg, uint16_t data); //Writes data into a double (two 8-bit) registers and checks to make sure that the data was written successfully. Returns 0 on no error, 1 on I2C write fail, and 2 if the register doesn't read back the same value that was written.
};
#endif