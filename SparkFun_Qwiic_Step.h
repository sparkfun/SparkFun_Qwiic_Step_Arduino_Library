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
    bool stop();
    bool setMaxSpeed(float speed);
    bool setSpeed(float speed);
    bool setAcceleration(float acceleration);
    bool moveTo(signed long absolute);
    bool move(signed long relative);
    bool setStepMode(uint8_t mode);
    bool setCurrentPosition(signed long pos);

    byte getStatus();
    float getMaxSpeed();
    float getSpeed();
    float getAcceleration();
    signed long getMove();
    signed long getMoveTo();
    signed long getCurrentPosition();
    signed long getDistanceToGo();
    uint8_t getStepMode();

    //Microstepping
    bool fullStepMode();
    bool halfStepMode();
    bool quarterStepMode();
    bool eighthStepMode();
    bool sixteenthStepMode();

    //Interrupt Handling
    bool clearInterrupts();
    bool disableInterrupts();
    bool enableIsLimitedInterrupt();
    bool disableIsLimitedInterrupt();
    bool clearIsLimited();
    bool enableStopWhenLimSwitchPressed();
    bool disableStopWhenLimSwitchPressed();
    bool enableDisableMotorWhenPosReached();
    bool disableDisableMotorWhenPosReached();
    bool clearEStop();

    bool isReached(); //Returns true if isReached bit is set
    bool clearIsReached();
    bool enableIsReachedInterrupt();
    bool disableIsReachedInterrupt();
    bool isRunning();
    bool isAccelerating();
    bool isDecelerating();
    bool isLimited();
    bool isEStopped();

    //Device control
    bool modeRun();
    bool modeRunSpeed();
    bool modeRunSpeedToPosition();
    bool modeStop();
    bool disableOutputs();
    bool enableOutputs();

    //Hold and run current
    bool setHoldCurrent(uint16_t current);
    bool setRunCurrent(uint16_t current);

    uint16_t getHoldCurrent();
    uint16_t getRunCurrent();

    //Internal I2C Abstraction
    bool read(Qwiic_Step_Register reg, uint8_t *buff, uint8_t buffSize);
    bool read(Qwiic_Step_Register reg, uint8_t data);
    bool write(Qwiic_Step_Register reg, uint8_t *buff, uint8_t buffSize);
    bool write(Qwiic_Step_Register reg, uint8_t data);
};
#endif