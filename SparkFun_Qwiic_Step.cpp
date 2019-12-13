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
    return (write(SET_SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed)));
}

bool QwiicStep::QSetAcceleration(float acceleration)
{
    return (write(ACCELERATION, (uint8_t *)&acceleration, (uint8_t)sizeof(acceleration)));
}

bool QwiicStep::QMoveTo(long absolute)
{
    return (write(MOVE_TO, (uint8_t *)&absolute, (uint8_t)sizeof(absolute)));
}

//QMoveTo

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
    read(SET_SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed));
    return speed;
}

float QwiicStep::QGetAcceleration()
{
    float acceleration;
    read(ACCELERATION, (uint8_t *)&acceleration, (uint8_t)sizeof(acceleration));
    return acceleration;
}

uint8_t QwiicStep::QGetStepMode()
{
    uint8_t mode;
    read(DEVICE_CONFIG, (uint8_t *)&mode, (uint8_t)sizeof(mode));
    return mode;
}

/*------------------------- Motor Stepping ------------------------------*/

void FullStepMode() {}

void HalfStepMode() {}

void QuarterStepMode() {}

void EighthStepMode() {}

void SixteenthStepMode() {}
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
//Use when just writing one byte of data
bool QwiicStep::write(Qwiic_Step_Register reg, uint8_t data)
{
    return (write(reg, (uint8_t *)&data, (uint8_t)sizeof(data)));
}