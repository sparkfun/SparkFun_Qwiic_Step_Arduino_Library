#include <SparkFun_Qwiic_Step.h>

/*--------------------------- Device Status -----------------------------*/
bool QwiicStep::begin(uint8_t address, TwoWire &wirePort)
{
    _deviceAddress = address; //grab the address that the motor is on
    _i2cPort = &wirePort;     //grab the port that the user wants to use

    //Setup motor with default step size, speed, maxSpeed and acceleration
    //So that user can simply .move() from their sketch
    fullStepMode();
    setMaxSpeed(800); //4 full rotations per second on a 200 step motor
    //setSpeed(300); //Only set if we are in runSpeed mode, otherwise library causes motor to twitch slowly.
    setAcceleration(200);

    //return true if the device is connected and the device ID is what we expect
    return (isConnected());
}

//Returns true if device answers with expected WHOAMI ID
bool QwiicStep::isConnected()
{
    _i2cPort->beginTransmission(_deviceAddress);
    if (_i2cPort->endTransmission() == 0)
    {
        if (checkDeviceID() == true)
            return true;
    }
    return false;
}

uint8_t QwiicStep::deviceID()
{
    uint8_t id;
    read(QS_ID, (uint8_t *)&id, (uint8_t)sizeof(id)); //read and return the value in the ID register
    return id;
}

bool QwiicStep::checkDeviceID()
{
    return (deviceID() == DEV_ID); //Return true if the device ID matches
}

uint16_t QwiicStep::getFirmwareVersion()
{
    uint16_t version;
    read(QS_FIRMWARE, (uint8_t *)&version, (uint8_t)sizeof(version));
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

    bool success = write(QS_I2C_ADDRESS, (uint8_t *)&address, (uint8_t)sizeof(address));

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

//Writes the move and moveTo registers to 0
//This causes the motor to spin down using the accel and maxSpeed parameters
bool QwiicStep::stop()
{
    bool status = true;
    signed long stopValue = 0; //Force a four byte write to register
    status &= write(QS_MOVE, (uint8_t *)&stopValue, (uint8_t)sizeof(stopValue));
    status &= write(QS_MOVE_TO, (uint8_t *)&stopValue, (uint8_t)sizeof(stopValue));
    return (status);
}

bool QwiicStep::setMaxSpeed(float speed)
{
    return (write(QS_MAX_SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed)));
}

bool QwiicStep::setSpeed(float speed)
{
    return (write(QS_SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed)));
}

bool QwiicStep::setAcceleration(float acceleration)
{
    return (write(QS_ACCELERATION, (uint8_t *)&acceleration, (uint8_t)sizeof(acceleration)));
}

bool QwiicStep::move(signed long relative)
{
    return (write(QS_MOVE, (uint8_t *)&relative, (uint8_t)sizeof(relative)));
}

//Returns the value currently in the Move register
signed long QwiicStep::getMove()
{
    signed long moveValue;
    read(QS_MOVE, (uint8_t *)&moveValue, (uint8_t)sizeof(moveValue));
    return (moveValue);
}

bool QwiicStep::moveTo(signed long absolute)
{
    return (write(QS_MOVE_TO, (uint8_t *)&absolute, (uint8_t)sizeof(absolute)));
}

signed long QwiicStep::getMoveTo()
{
    signed long moveToValue;
    read(QS_MOVE_TO, (uint8_t *)&moveToValue, (uint8_t)sizeof(moveToValue));
    return (moveToValue);
}

bool QwiicStep::setStepMode(uint8_t mode)
{
    return (write(QS_DEVICE_CONFIG, mode));
}

bool QwiicStep::setCurrentPosition(signed long pos)
{
    return (write(QS_CURRENT_POSITION, (uint8_t *)&pos, (uint8_t)sizeof(pos)));
}

float QwiicStep::getMaxSpeed()
{
    float speed;
    read(QS_MAX_SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed));
    return speed;
}

float QwiicStep::getSpeed()
{
    float speed;
    read(QS_SPEED, (uint8_t *)&speed, (uint8_t)sizeof(speed));
    return speed;
}

float QwiicStep::getAcceleration()
{
    float acceleration;
    read(QS_ACCELERATION, (uint8_t *)&acceleration, (uint8_t)sizeof(acceleration));
    return acceleration;
}

//Returns the current status register
uint8_t QwiicStep::getStatus()
{
    uint8_t stat;
    read(QS_STATUS, (uint8_t *)&stat, (uint8_t)sizeof(stat));
    return stat;
}

signed long QwiicStep::getCurrentPosition()
{
    signed long pos;
    read(QS_CURRENT_POSITION, (uint8_t *)&pos, (uint8_t)sizeof(pos));
    return pos;
}

signed long QwiicStep::getDistanceToGo()
{
    signed long dist;
    read(QS_DIST_TO_GO, (uint8_t *)&dist, (uint8_t)sizeof(dist));
    return dist;
}

//Returns the bits set that control the micro step amount
uint8_t QwiicStep::getStepMode()
{
    uint8_t mode;
    read(QS_DEVICE_CONFIG, (uint8_t *)&mode, (uint8_t)sizeof(mode));
    return mode;
}

/*------------------------- Motor Stepping ------------------------------*/

bool QwiicStep::fullStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 0;
    deviceConfigure.MS2 = 0;
    deviceConfigure.MS3 = 0;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::halfStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 1;
    deviceConfigure.MS2 = 0;
    deviceConfigure.MS3 = 0;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::quarterStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 0;
    deviceConfigure.MS2 = 1;
    deviceConfigure.MS3 = 0;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::eighthStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 1;
    deviceConfigure.MS2 = 1;
    deviceConfigure.MS3 = 0;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::sixteenthStepMode()
{
    deviceConfigBitField deviceConfigure;
    deviceConfigure.MS1 = 1;
    deviceConfigure.MS2 = 1;
    deviceConfigure.MS3 = 1;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

/*---------------------- Interrupt Configuration ------------------------*/

//We must clear all bits that can cause the interrupt pin to go low
//See interruptConfigBitField for possible related bits
bool QwiicStep::clearInterrupts()
{
    statusRegisterBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    status.isLimited = 0;
    status.isReached = 0;
    return (write(QS_STATUS, status.byteWrapped));
}

bool QwiicStep::disableInterrupts()
{
    interruptConfigBitField intConfig;
    read(QS_INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.isReachedInterruptEnable = 0;
    intConfig.isLimitedInterruptEnable = 0;
    return (write(QS_INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::enableIsReachedInterrupt()
{
    interruptConfigBitField intConfig;
    read(QS_INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.isReachedInterruptEnable = 1;
    return (write(QS_INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::disableIsReachedInterrupt()
{
    interruptConfigBitField intConfig;
    read(QS_INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.isReachedInterruptEnable = 0;
    return (write(QS_INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::clearIsReached()
{
    statusRegisterBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    status.isReached = 0;
    return (write(QS_STATUS, status.byteWrapped));
}

bool QwiicStep::isReached()
{
    statusRegisterBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isReached == true)
        return (true);
    return (false);
}

bool QwiicStep::isRunning()
{
    statusRegisterBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isRunning == true)
        return (true);
    return (false);
}

bool QwiicStep::isAccelerating()
{
    statusRegisterBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isAccelerating == true)
        return (true);
    return (false);
}

bool QwiicStep::isDecelerating()
{
    statusRegisterBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isDecelerating == true)
        return (true);
    return (false);
}

bool QwiicStep::isLimited()
{
    statusRegisterBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isLimited == true)
        return (true);
    return (false);
}

bool QwiicStep::isEStopped()
{
    statusRegisterBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isEStopped == true)
        return (true);
    return (false);
}

bool QwiicStep::enableIsLimitedInterrupt()
{
    interruptConfigBitField intConfig;
    read(QS_INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.isLimitedInterruptEnable = 1;
    return (write(QS_INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::disableIsLimitedInterrupt()
{
    interruptConfigBitField intConfig;
    read(QS_INTERRUPT_CONFIG, (uint8_t *)&intConfig.byteWrapped, sizeof(intConfig.byteWrapped));
    intConfig.isLimitedInterruptEnable = 0;
    return (write(QS_INTERRUPT_CONFIG, intConfig.byteWrapped));
}

bool QwiicStep::clearIsLimited()
{
    statusRegisterBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    status.isLimited = 0;
    return (write(QS_STATUS, status.byteWrapped));
}

bool QwiicStep::enableStopWhenLimSwitchPressed()
{
    deviceConfigBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.stopOnLimitSwitchPress = 1;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::disableStopWhenLimSwitchPressed()
{
    deviceConfigBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.stopOnLimitSwitchPress = 0;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::enableDisableMotorWhenPosReached()
{
    deviceConfigBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.disableMotorOnPositionReached = 1;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::disableDisableMotorWhenPosReached()
{
    deviceConfigBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.disableMotorOnPositionReached = 0;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

//User needs to manually clear eStop bit when an emergency stop has occurred
//DEBUG: still need to test this
bool QwiicStep::clearEStop()
{
    statusRegisterBitField status;
    status.isEStopped = 0;
    return (write(QS_STATUS, status.byteWrapped));
}

//DEBUG: still need to test all of these
/*---------------------------- Run options ------------------------------*/

bool QwiicStep::modeRun()
{
    motorControlBitField motorControl;
    read(QS_MOTOR_CONTROL, (uint8_t *)&motorControl.byteWrapped, sizeof(motorControl.byteWrapped));
    motorControl.run = 1;
    motorControl.runSpeed = 0;
    motorControl.runSpeedToPosition = 0;
    motorControl.stop = 0;
    return (write(QS_MOTOR_CONTROL, motorControl.byteWrapped));
}

bool QwiicStep::modeRunSpeed()
{
    motorControlBitField motorControl;
    read(QS_MOTOR_CONTROL, (uint8_t *)&motorControl.byteWrapped, sizeof(motorControl.byteWrapped));
    motorControl.run = 0;
    motorControl.runSpeed = 1;
    motorControl.runSpeedToPosition = 0;
    motorControl.stop = 0;
    return (write(QS_MOTOR_CONTROL, motorControl.byteWrapped));
}

bool QwiicStep::modeRunSpeedToPosition()
{
    motorControlBitField motorControl;
    read(QS_MOTOR_CONTROL, (uint8_t *)&motorControl.byteWrapped, sizeof(motorControl.byteWrapped));
    motorControl.run = 0;
    motorControl.runSpeed = 0;
    motorControl.runSpeedToPosition = 1;
    motorControl.stop = 0;
    return (write(QS_MOTOR_CONTROL, motorControl.byteWrapped));
}

bool QwiicStep::modeStop()
{
    motorControlBitField motorControl;
    read(QS_MOTOR_CONTROL, (uint8_t *)&motorControl.byteWrapped, sizeof(motorControl.byteWrapped));
    motorControl.run = 0;
    motorControl.runSpeed = 0;
    motorControl.runSpeedToPosition = 0;
    motorControl.stop = 1;
    return (write(QS_MOTOR_CONTROL, motorControl.byteWrapped));
}

bool QwiicStep::disableOutputs()
{
    motorControlBitField motorControl;
    read(QS_MOTOR_CONTROL, (uint8_t *)&motorControl.byteWrapped, sizeof(motorControl.byteWrapped));
    motorControl.disableMotor = 1;
    return (write(QS_MOTOR_CONTROL, motorControl.byteWrapped));
}

bool QwiicStep::enableOutputs()
{
    motorControlBitField motorControl;
    read(QS_MOTOR_CONTROL, (uint8_t *)&motorControl.byteWrapped, sizeof(motorControl.byteWrapped));
    motorControl.disableMotor = 0;
    return (write(QS_MOTOR_CONTROL, motorControl.byteWrapped));
}

/*------------------------- Configure Current ---------------------------*/

bool QwiicStep::setHoldCurrent(uint16_t current)
{
    return (write(QS_HOLD_CURRENT, (uint8_t *)&current, (uint8_t)sizeof(current)));
}

bool QwiicStep::setRunCurrent(uint16_t current)
{
    return (write(QS_RUN_CURRENT, (uint8_t *)&current, (uint8_t)sizeof(current)));
}

uint16_t QwiicStep::getHoldCurrent()
{
    uint16_t holdCurrent;
    read(QS_HOLD_CURRENT, (uint8_t *)&holdCurrent, (uint8_t)sizeof(holdCurrent));
    return holdCurrent;
}

uint16_t QwiicStep::getRunCurrent()
{
    uint16_t runCurrent;
    read(QS_RUN_CURRENT, (uint8_t *)&runCurrent, (uint8_t)sizeof(runCurrent));
    return runCurrent;
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