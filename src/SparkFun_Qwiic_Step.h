#ifndef __SparkFun_Qwiic_Step_H__
#define __SparkFun_Qwiic_Step_H__

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
    uint8_t getDeviceID();                                                   //Returns the 8-bit device ID of the attached device.
    uint16_t getFirmwareVersion();                                           //Returns the firmware version of the attched device as a 16-bit integer. The leftmost (high) byte is the major revision number and the rightmost (low) byte is the minor revision number.
    bool setI2Caddress(uint8_t address);                                     //Configures the attach to the I2C bus using the specified address
    uint8_t getI2Caddress();                                                 //Returns the I2C address of the device
    byte getStatus();

    //Motor control
    bool stop();
    bool hardStop();
    bool setMaxSpeed(float speed);
    float getMaxSpeed();
    bool setSpeed(float speed);
    float getSpeed();
    bool setAcceleration(float acceleration);
    float getAcceleration();
    bool setPosition(int32_t newPosition);
    int32_t getPosition();
    bool moveTo(long absolute);
    long getMoveTo();
    bool move(long relative);
    long getMove();
    long toGo();
    bool enable();  //Energize the coils
    bool disable(); //Turn off coils

    bool setStepSize(sfe_qs_step_size stepSize);
    uint8_t getStepSize();

    //Interrupt Handling
    bool clearInterrupts();
    bool disableInterrupts();
    bool enableIsLimitedInterrupt();
    bool disableIsLimitedInterrupt();
    bool clearIsLimited();
    bool enableStopWhenLimitSwitchPressed();
    bool disableStopWhenLimitSwitchPressed();
    bool enableDisableMotorWhenPosReached();
    bool disableDisableMotorWhenPosReached();
    bool enableDisableMotorOnEStop();
    bool disableDisableMotorOnEStop();
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
    bool setMode(sfe_qs_mode modeType);
    bool setModeRunWithAcceleration();
    bool setModeRunContinuous();
    bool setModeRunToPosition();

    //Special record to NVM functions
    bool recordMoveToNVM();
    bool recordSpeedToNVM();

    bool setHoldCurrent(uint16_t maxCurrent);
    uint16_t getHoldCurrent();
    bool setRunCurrent(uint16_t maxCurrent);
    uint16_t getRunCurrent();

    //Internal I2C Abstraction
    bool read(sfe_qs_register reg, uint8_t *buff, uint8_t buffSize);
    bool read(sfe_qs_register reg, uint8_t data);
    bool write(sfe_qs_register reg, uint8_t *buff, uint8_t buffSize);
    bool write(sfe_qs_register reg, uint8_t data);
};
#endif