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
    Serial.print("The device address is: 0x");
    Serial.println(_deviceAddress, HEX);
    _i2cPort = &wirePort; //grab the port that the user wants to use
    readQuadRegister(ACCELERATION);

    readSingleRegister(38);
    // // getAddress();
    // //return true if the device is connected and the device ID is what we expect
    return (checkDeviceID());
}

bool QwiicStep::isConnected()
{
    _i2cPort->beginTransmission(_deviceAddress);
    if (_i2cPort->endTransmission() == 0)
    {
        Serial.println("Hello transmission successful!");
        return true;
    }
    Serial.println("Hello transmission unsuccessful!");
    return false;
}

//FOR DEBUGGING
uint8_t QwiicStep::getAddress()
{
    return readSingleRegister(I2C_ADDRESS);
}

uint8_t QwiicStep::deviceID()
{
    return readSingleRegister(ID); //read and return the value in the ID register
}

bool QwiicStep::checkDeviceID()
{
    return (deviceID() == DEV_ID); //Return true if the device ID matches
}

uint16_t QwiicStep::getFirmwareVersion()
{
    uint16_t version = (readSingleRegister(FIRMWARE_MSB)) << 8;
    version |= readSingleRegister(FIRMWARE_LSB);
    return version;
}

/*---------------------- Internal I2C Abstraction -----------------------*/

uint8_t QwiicStep::readSingleRegister(Qwiic_Step_Register reg)
{
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->endTransmission();

    //typecasting the 1 parameter in requestFrom so that the compiler
    //doesn't give us a warning about multiple candidates
    if (_i2cPort->requestFrom(_deviceAddress, static_cast<uint8_t>(1)) != 0)
    {
        Serial.println("Hello, I'm about to read the register!");
        return _i2cPort->read();
    }
    Serial.println("I don't think I read the register!");
    return 0;
}

uint16_t QwiicStep::readDoubleRegister(Qwiic_Step_Register reg)
{ //little endian
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->endTransmission();

    //typecasting the 2 parameter in requestFrom so that the compiler
    //doesn't give us a warning about multiple candidates
    if (_i2cPort->requestFrom(_deviceAddress, static_cast<uint8_t>(2)) != 0)
    {
        uint16_t data = _i2cPort->read();
        data |= (_i2cPort->read() << 8);
        return data;
    }
    return 0;
}

unsigned long QwiicStep::readQuadRegister(Qwiic_Step_Register reg)
{
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->endTransmission();

    union databuffer {
        uint8_t array[4];
        unsigned long integer;
    };

    databuffer data;

    //typecasting the 4 parameter in requestFrom so that the compiler
    //doesn't give us a warning about multiple candidates
    if (_i2cPort->requestFrom(_deviceAddress, static_cast<uint8_t>(4)) != 0)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            data.array[i] = _i2cPort->read();
        }
    }
    return data.integer;
}

bool QwiicStep::writeSingleRegister(Qwiic_Step_Register reg, uint8_t data)
{
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->write(data);
    if (_i2cPort->endTransmission() == 0)
        return true;
    return false;
}

bool QwiicStep::writeDoubleRegister(Qwiic_Step_Register reg, uint16_t data)
{
    _i2cPort->beginTransmission(_deviceAddress);
    _i2cPort->write(reg);
    _i2cPort->write(lowByte(data));
    _i2cPort->write(highByte(data));
    if (_i2cPort->endTransmission() == 0)
        return true;
    return false;
}

uint8_t QwiicStep::writeSingleRegisterWithReadback(Qwiic_Step_Register reg, uint8_t data)
{
    if (writeSingleRegister(reg, data))
        return 1;
    if (readSingleRegister(reg) != data)
        return 2;
    return 0;
}

uint16_t QwiicStep::writeDoubleRegisterWithReadback(Qwiic_Step_Register reg, uint16_t data)
{
    if (writeDoubleRegister(reg, data))
        return 1;
    if (readDoubleRegister(reg) != data)
        return 2;
    return 0;
}