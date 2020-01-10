#include <Wire.h>
#include <SparkFun_Qwiic_Step.h>

//DEBUG: not sure what this is...
#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*--------------------------- Device Status -----------------------------*/
bool QwiicStep::begin(uint8_t address, TwoWire &wirePort)
{
    _deviceAddress = address; //grab the address that the motor is on
    _i2cPort = &wirePort;     //grab the port that the user wants to use

    //return true if the device is connected and the device ID is what we expect
    return (isConnected() & checkDeviceID());
}

bool QwiicStep::isConnected()
{
    _i2cPort->beginTransmission(_deviceAddress);
    if (_i2cPort->endTransmission() == 0)
    {
        return true;
    }
    return false;
}

uint8_t QwiicStep::deviceID()
{
    uint8_t id;
    read(ID, (uint8_t *)&id, (uint8_t)sizeof(id)); //read and return the value in the ID register
    return id;
}

bool QwiicStep::checkDeviceID()
{
    return (deviceID() == DEV_ID); //Return true if the device ID matches
}

uint16_t QwiicStep::getFirmwareVersion()
{
    uint16_t version;
    read(FIRMWARE, (uint8_t *)&version, (uint8_t)sizeof(version));
    return version;
}

//DEBUG: have to test this in the future (once EEPROM is set up)
bool QwiicStep::setI2Caddress(uint8_t address)
{
    //check that address is valid
    if (address < 0x08 || address > 0x77)
    {
        return 1; //error immediately if the address is out of legal range
    }

    bool success = write(I2C_ADDRESS, (uint8_t *)&address, (uint8_t)sizeof(address));

    if (success == true)
    {
        _deviceAddress = address;
        return true;
    }
    else
    {
        return false;
    }
}

uint8_t QwiicStep::getI2Caddress()
{
    return _deviceAddress;
}

/*-------------------------- Motor Control ------------------------------*/

bool QwiicStep::QSetMaxSpeed(float speed)
{
    return (write(MAX_SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed)));
}

bool QwiicStep::QSetSpeed(float speed)
{
    return (write(SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed)));
}

bool QwiicStep::QSetAcceleration(float acceleration)
{
    return (write(ACCELERATION, (uint8_t *)&acceleration, (uint8_t)sizeof(acceleration)));
}

bool QwiicStep::QMoveTo(long absolute)
{
    return (write(MOVE_TO, (uint8_t *)&absolute, (uint8_t)sizeof(absolute)));
}

bool QwiicStep::QMove(long relative)
{
    return (write(MOVE, (uint8_t *)&relative, (uint8_t)sizeof(relative)));
}

bool QwiicStep::QSetStepMode(uint8_t mode)
{
    return (write(DEVICE_CONFIG, mode));
}

float QwiicStep::QGetMaxSpeed()
{
    float speed;
    read(MAX_SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed));
    return speed;
}

float QwiicStep::QGetSpeed()
{
    float speed;
    read(SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed));
    return speed;
}

float QwiicStep::QGetAcceleration()
{
    float acceleration;
    read(ACCELERATION, (uint8_t *)&acceleration, (uint8_t)sizeof(acceleration));
    return acceleration;
}

//DEBUG-- not sure what this is supposed to be yettttt
uint8_t QwiicStep::QGetStepMode()
{
    uint8_t mode;
    read(DEVICE_CONFIG, (uint8_t *)&mode, (uint8_t)sizeof(mode));
    return mode;
}

/*------------------------- Motor Stepping ------------------------------*/

bool QwiicStep::fullStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 0;
    deviceConfigure.MS2 = 0;
    deviceConfigure.MS3 = 0;
    return (write(DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::halfStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 1;
    deviceConfigure.MS2 = 0;
    deviceConfigure.MS3 = 0;
    return (write(DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::quarterStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 0;
    deviceConfigure.MS2 = 1;
    deviceConfigure.MS3 = 0;
    return (write(DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::eighthStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 1;
    deviceConfigure.MS2 = 1;
    deviceConfigure.MS3 = 0;
    return (write(DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::sixteenthStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 1;
    deviceConfigure.MS2 = 1;
    deviceConfigure.MS3 = 1;
    return (write(DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

/*---------------------- Interrupt Configuration ------------------------*/

bool QwiicStep::enablePositionReachedInterrupt()
{
    interruptConfigBitField intConfig;
    read(INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.requestedPosReachedEnable = 1;
    return (write(INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::disablePositionReachedInterrupt()
{
    interruptConfigBitField intConfig;
    read(INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.requestedPosReachedEnable = 0;
    return (write(INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::clearIsReachedInterrupt()
{
    interruptConfigBitField intConfig;
    read(INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.requestedPosReachedIntTriggered = 0;
    return (write(INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::enableLimSwitchPressedInterrupt()
{
    interruptConfigBitField intConfig;
    read(INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.limSwitchPressedEnable = 1;
    return (write(INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::disableLimSwitchPressedInterrupt()
{
    interruptConfigBitField intConfig;
    read(INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.limSwitchPressedEnable = 0;
    return (write(INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::clearLimSwitchPressInterrupt()
{
    statusRegisterBitField status;
    read(STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    status.isLimited = 0;
    return (write(STATUS, status.byteWrapped));
}

bool QwiicStep::enableStopWhenLimSwitchPressed()
{
    deviceConfigBitField deviceConfigure;
    read(DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.stopOnLimitSwitchPress = 1;
    return (write(DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::disableStopWhenLimSwitchPressed()
{
    deviceConfigBitField deviceConfigure;
    read(DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.stopOnLimitSwitchPress = 0;
    return (write(DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::enableDisableMotorWhenPosReached()
{
    deviceConfigBitField deviceConfigure;
    read(DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.disableMotorPositionReached = 1;
    return (write(DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::disableDisableMotorWhenPosReached()
{
    deviceConfigBitField deviceConfigure;
    read(DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.disableMotorPositionReached = 0;
    return (write(DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

//User needs to manually clear eStop bit when an emergency stop has occurred
//DEBUG: still need to test this
bool QwiicStep::clearEStop()
{
    statusRegisterBitField status;
    status.eStopped = 0;
    return (write(STATUS, status.byteWrapped));
}

//DEBUG: still need to test all of these
/*---------------------------- Run options ------------------------------*/

bool QwiicStep::QRun()
{
    deviceControlBitField deviceControl;
    read(DEVICE_CONTROL, (uint8_t *)&deviceControl.byteWrapped, sizeof(deviceControl.byteWrapped));
    deviceControl.run = 1;
    deviceControl.runSpeed = 0;
    deviceControl.runSpeedToPosition = 0;
    deviceControl.stop = 0;
    return (write(DEVICE_CONTROL, deviceControl.byteWrapped));
}

bool QwiicStep::QRunSpeed()
{
    deviceControlBitField deviceControl;
    read(DEVICE_CONTROL, (uint8_t *)&deviceControl.byteWrapped, sizeof(deviceControl.byteWrapped));
    deviceControl.run = 0;
    deviceControl.runSpeed = 1;
    deviceControl.runSpeedToPosition = 0;
    deviceControl.stop = 0;
    return (write(DEVICE_CONTROL, deviceControl.byteWrapped));
}

bool QwiicStep::QRunSpeedToPosition()
{
    deviceControlBitField deviceControl;
    read(DEVICE_CONTROL, (uint8_t *)&deviceControl.byteWrapped, sizeof(deviceControl.byteWrapped));
    deviceControl.run = 0;
    deviceControl.runSpeed = 0;
    deviceControl.runSpeedToPosition = 1;
    deviceControl.stop = 0;
    return (write(DEVICE_CONTROL, deviceControl.byteWrapped));
}

bool QwiicStep::QStop()
{
    deviceControlBitField deviceControl;
    read(DEVICE_CONTROL, (uint8_t *)&deviceControl.byteWrapped, sizeof(deviceControl.byteWrapped));
    deviceControl.run = 0;
    deviceControl.runSpeed = 0;
    deviceControl.runSpeedToPosition = 0;
    deviceControl.stop = 1;
    return (write(DEVICE_CONTROL, deviceControl.byteWrapped));
}

bool QwiicStep::QDisableOutputs()
{
    deviceControlBitField deviceControl;
    read(DEVICE_CONTROL, (uint8_t *)&deviceControl.byteWrapped, sizeof(deviceControl.byteWrapped));
    deviceControl.disableMotor = 1;
    return (write(DEVICE_CONTROL, deviceControl.byteWrapped));
}

bool QwiicStep::QEnableOutputs()
{
    deviceControlBitField deviceControl;
    read(DEVICE_CONTROL, (uint8_t *)&deviceControl.byteWrapped, sizeof(deviceControl.byteWrapped));
    deviceControl.disableMotor = 0;
    return (write(DEVICE_CONTROL, deviceControl.byteWrapped));
}

/*---------------------- Internal I2C Abstraction -----------------------*/

bool QwiicStep::read(Qwiic_Step_Register reg, uint8_t *buff, uint8_t buffSize)
{
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->endTransmission();

    if (_i2cPort->requestFrom(_deviceAddress, buffSize) > 0)
    {
        for (uint8_t i = 0; i < buffSize; i++)
        {
            buff[i] = _i2cPort->read();
        }
        return true;
    }

    return false;
}

//Overloaded function declaration
//Use when reading just one byte of data
bool QwiicStep::read(Qwiic_Step_Register reg, uint8_t data)
{
    return (read(reg, (uint8_t *)&data, (uint8_t)sizeof(data)));
}

bool QwiicStep::write(Qwiic_Step_Register reg, uint8_t *buff, uint8_t buffSize)
{
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);

    for (uint8_t i = 0; i < buffSize; i++)
        _i2cPort->write(buff[i]);

    if (_i2cPort->endTransmission() == 0)
        return true;
    return false;
}

//Overloaded function declaration
//Use when writing just one byte of data
bool QwiicStep::write(Qwiic_Step_Register reg, uint8_t data)
{
    return (write(reg, (uint8_t *)&data, (uint8_t)sizeof(data)));
}