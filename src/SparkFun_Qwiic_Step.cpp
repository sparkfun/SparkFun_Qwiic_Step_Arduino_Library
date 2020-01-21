#include <SparkFun_Qwiic_Step.h>

/*--------------------------- Device Status -----------------------------*/
bool QwiicStep::begin(uint8_t address, TwoWire &wirePort)
{
    _deviceAddress = address; //grab the address that the motor is on
    _i2cPort = &wirePort;     //grab the port that the user wants to use

    if (isConnected() == false)
        return (false);

    //Setup motor with default mode, step size, maxSpeed and acceleration
    //So that user can simply .move() from their sketch (no additional config needed)
    setMode(QS_MODE_RUN_TO_POSITION);

    setStepSize(STEPSIZE_FULL);

    setMaxSpeed(800); //4 full rotations per second on a 200 step motor
    setSpeed(200);

    setPosition(0); //Mark our current position as zero point

    enable(); //Make sure outputs are enabled by default

    return (true); //return true if the device is connected and the device ID is what we expect
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

bool QwiicStep::setPosition(int32_t newPosition)
{
    return (write(QS_CURRENT_POSITION, (uint8_t *)&newPosition, (uint8_t)sizeof(newPosition)));
}

bool QwiicStep::move(long relative)
{
    return (write(QS_MOVE, (uint8_t *)&relative, (uint8_t)sizeof(relative)));
}

long QwiicStep::getMove()
{
    long moveValue;
    read(QS_MOVE, (uint8_t *)&moveValue, (uint8_t)sizeof(moveValue));
    return (moveValue);
}

bool QwiicStep::moveTo(long absolute)
{
    return (write(QS_MOVE_TO, (uint8_t *)&absolute, (uint8_t)sizeof(absolute)));
}

long QwiicStep::getMoveTo()
{
    long moveToValue;
    read(QS_MOVE_TO, (uint8_t *)&moveToValue, (uint8_t)sizeof(moveToValue));
    return (moveToValue);
}

long QwiicStep::toGo()
{
    long distanceToGo;
    read(QS_DIST_TO_GO, (uint8_t *)&distanceToGo, (uint8_t)sizeof(distanceToGo));
    return (distanceToGo);
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

uint8_t QwiicStep::getStatus()
{
    uint8_t stat;
    read(QS_STATUS, (uint8_t *)&stat, (uint8_t)sizeof(stat));
    return stat;
}

bool QwiicStep::setHoldCurrent(int16_t newCurrent)
{
    return (write(QS_HOLD_CURRENT, (uint8_t *)&newCurrent, (uint8_t)sizeof(newCurrent)));
}

uint16_t QwiicStep::getHoldCurrent()
{
    uint16_t holdCurrent;
    read(QS_HOLD_CURRENT, (uint8_t *)&holdCurrent, (uint8_t)sizeof(holdCurrent));
    return (holdCurrent);
}

bool QwiicStep::setRunCurrent(int16_t newCurrent)
{
    return (write(QS_RUN_CURRENT, (uint8_t *)&newCurrent, (uint8_t)sizeof(newCurrent)));
}

uint16_t QwiicStep::getRunCurrent()
{
    uint16_t runCurrent;
    read(QS_RUN_CURRENT, (uint8_t *)&runCurrent, (uint8_t)sizeof(runCurrent));
    return (runCurrent);
}

/*------------------------- Motor Stepping ------------------------------*/

bool QwiicStep::setStepSize(sfe_qs_step_size stepSize)
{
    configBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.MS1 = stepSize & 0b001;
    deviceConfigure.MS2 = stepSize & 0b010;
    deviceConfigure.MS3 = stepSize & 0b100;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

uint8_t QwiicStep::getStepSize()
{
    uint8_t mode;
    read(QS_DEVICE_CONFIG, (uint8_t *)&mode, (uint8_t)sizeof(mode));
    return mode;
}

/*---------------------- Interrupt Configuration ------------------------*/

//We must clear all bits that can cause the interrupt pin to go low
//See interruptConfigBitField for possible related bits
bool QwiicStep::clearInterrupts()
{
    statusBitField status;
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
    statusBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    status.isReached = 0;
    return (write(QS_STATUS, status.byteWrapped));
}

bool QwiicStep::isReached()
{
    statusBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isReached == true)
        return (true);
    return (false);
}

bool QwiicStep::isRunning()
{
    statusBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isRunning == true)
        return (true);
    return (false);
}

bool QwiicStep::isAccelerating()
{
    statusBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isAccelerating == true)
        return (true);
    return (false);
}

bool QwiicStep::isDecelerating()
{
    statusBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isDecelerating == true)
        return (true);
    return (false);
}

bool QwiicStep::isLimited()
{
    statusBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    if (status.isLimited == true)
        return (true);
    return (false);
}

bool QwiicStep::isEStopped()
{
    statusBitField status;
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
    statusBitField status;
    read(QS_STATUS, (uint8_t *)&status.byteWrapped, sizeof(status.byteWrapped));
    status.isLimited = 0;
    return (write(QS_STATUS, status.byteWrapped));
}

bool QwiicStep::enableStopWhenLimitSwitchPressed()
{
    configBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.stopOnLimitSwitchPress = 1;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::disableStopWhenLimitSwitchPressed()
{
    configBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.stopOnLimitSwitchPress = 0;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::enableDisableMotorWhenPosReached()
{
    configBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.disableMotorOnPositionReached = 1;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::disableDisableMotorWhenPosReached()
{
    configBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.disableMotorOnPositionReached = 0;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::enableDisableMotorOnEStop()
{
    configBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.disableMotorOnEStop = 1;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

bool QwiicStep::disableDisableMotorOnEStop()
{
    configBitField deviceConfigure;
    read(QS_DEVICE_CONFIG, (uint8_t *)&deviceConfigure.byteWrapped, sizeof(deviceConfigure.byteWrapped));
    deviceConfigure.disableMotorOnEStop = 0;
    return (write(QS_DEVICE_CONFIG, deviceConfigure.byteWrapped));
}

//User needs to manually clear eStop bit when an emergency stop has occurred
bool QwiicStep::clearEStop()
{
    statusBitField status;
    status.isEStopped = 0;
    return (write(QS_STATUS, status.byteWrapped));
}

/*---------------------------- Run options ------------------------------*/

bool QwiicStep::setMode(sfe_qs_mode modeType)
{
    controlBitField motorControl;
    read(QS_MOTOR_CONTROL, (uint8_t *)&motorControl.byteWrapped, sizeof(motorControl.byteWrapped));
    motorControl.byteWrapped &= 0b11110000;
    motorControl.byteWrapped |= modeType;
    return (write(QS_MOTOR_CONTROL, motorControl.byteWrapped));
}

bool QwiicStep::setModeRunToPosition()
{
    return (setMode(QS_MODE_RUN_TO_POSITION));
}

bool QwiicStep::setModeRunWithAcceleration()
{
    return (setMode(QS_MODE_RUN_TO_POSITION_WITH_ACCELERATION));
}

bool QwiicStep::setModeRunContinuous()
{
    return (setMode(QS_MODE_RUN_CONTINUOUS));
}

bool QwiicStep::stop()
{
    return (setMode(QS_MODE_HARD_STOP));
}

bool QwiicStep::disable()
{
    controlBitField motorControl;
    read(QS_MOTOR_CONTROL, (uint8_t *)&motorControl.byteWrapped, sizeof(motorControl.byteWrapped));
    motorControl.disableMotor = 1;
    return (write(QS_MOTOR_CONTROL, motorControl.byteWrapped));
}

bool QwiicStep::enable()
{
    controlBitField motorControl;
    read(QS_MOTOR_CONTROL, (uint8_t *)&motorControl.byteWrapped, sizeof(motorControl.byteWrapped));
    motorControl.disableMotor = 0;
    return (write(QS_MOTOR_CONTROL, motorControl.byteWrapped));
}

bool QwiicStep::recordMoveToNVM()
{
    return (write(QS_UNLOCK_MOVE_NVM, 0x59));
}

bool QwiicStep::recordSpeedToNVM()
{
    return (write(QS_UNLOCK_SPEED_NVM, 0xC4));
}

/*---------------------- Internal I2C Abstraction -----------------------*/

bool QwiicStep::read(sfe_qs_register reg, uint8_t *buff, uint8_t buffSize)
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
bool QwiicStep::read(sfe_qs_register reg, uint8_t data)
{
    return (read(reg, (uint8_t *)&data, (uint8_t)sizeof(data)));
}

bool QwiicStep::write(sfe_qs_register reg, uint8_t *buff, uint8_t buffSize)
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
bool QwiicStep::write(sfe_qs_register reg, uint8_t data)
{
    return (write(reg, (uint8_t *)&data, (uint8_t)sizeof(data)));
}