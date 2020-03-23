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
    bool stop();    //Brings the motor to a hard stop in the midst of movement
    bool setMaxSpeed(float speed);  //Set the max speed of the motor when it's moving with acceleration 
    float getMaxSpeed();    //Returns the max speed of the motor 
    bool setSpeed(float speed); //Set the speed of the motor without acceleration. Use this with runContinuous or runToPosition
    float getSpeed();   //Returns the speed of the motor
    bool setAcceleration(float acceleration);   //Set the motor acceleration
    float getAcceleration();    //Returns the motor acceleration
    bool setPosition(int32_t newPosition);  //Set the current position of the motor (in steps)
    int32_t getPosition();  //Returns the current position of the motor (in steps)
    bool moveTo(long absolute); //Set the absolute target position for the motor to move to 
    long getMoveTo();   //Returns the absolute target position for the motor to move to
    bool move(long relative);   //Set the relative target position for the motor to move
    long getMove(); //Returns the relative target position for the motor to move 
    long toGo();    //Returns the number of steps left to go
    bool enable();  //Energize the coils
    bool disable(); //Turn off coils

    //Micro-stepping
    bool setStepSize(sfe_qs_step_size stepSize);    //Set the step size of the motor. Options are full, half, quarter, eighth, and sixteenth of a step.
    uint8_t getStepSize();  //Returns the current step size.

    //Motor status
    bool isReached(); //Returns true if isReached bit is set
    bool isRunning();   //Returns true if isRunning bit is set
    bool isAccelerating();  //Returns true if isAccelerating bit is set
    bool isDecelerating();  //Returns true if isDecelerating bit is set
    bool isLimited();   //Returns true if isLimited bit is set
    bool isEStopped();  //Returns true if isEStopped bit is set

    //Interrupt Handling
    bool clearInterrupts(); //User needs to clear all the interrupt bits (isLimited, isReached) once an interrupt has triggered
    bool disableInterrupts();   //Disable ALL interrupts
    bool enableIsReachedInterrupt();    //Enable isReached interrupt which triggers when the motor reaches its destination
    bool disableIsReachedInterrupt();   //Disable the isReached interrupt
    bool clearIsReached();  //Clear the isReached bit
    bool enableIsLimitedInterrupt();    //Enable isLimited interrupt which triggers when the limit switch is pressed
    bool disableIsLimitedInterrupt();   //Disable isLimited interrupt
    bool clearIsLimited();  //Clear the isLimited bit
    bool enableStopWhenLimitSwitchPressed();    //Enable the motor to stop when isLimited is triggered
    bool disableStopWhenLimitSwitchPressed();   //Disable the motor stopping when isLimited is triggered
    bool enableDisableMotorWhenPosReached();    //Enable the functionality to disable the motor once the motor has reached its position  
    bool disableDisableMotorWhenPosReached();   //Disable the functionality to disable the motor once the motor has reached its position
    bool enableDisableMotorOnEStop();   //Enable the functionality to disable the motor when EStop is pressed
    bool disableDisableMotorOnEStop();  //Disable the functionality to disable the motor when EStop is pressed
    bool clearEStop();  //Clear the isEStopped bit

    //Device control
    bool setMode(sfe_qs_mode modeType); //Set the run mode of the motor. Options are run to position, run to position with acceleration, run continuous, and hard stop.
    bool setModeRunWithAcceleration();  //This mode runs the motor to the desired position with acceleration. Need to set move or moveTo, maxSpeed, and acceleration before calling this run mode.
    bool setModeRunContinuous();    //This mode runs the motor at a continuous speed until it is told to stop. Need to set the speed and maxSpeed before calling this run mode.
    bool setModeRunToPosition();    //This mode runs the motor at a continuous speed to a desired position. Need to set the speed, maxSpeed, and either move or moveTo before calling this run mode.

    //Special record to NVM functions
    bool recordMoveToNVM(); //Records the current move value to non-volatile memory. Used for headless operation.
    bool recordSpeedToNVM();    //Records the current speed value to non-volatile memory. Used for headless operation.

    //Run/Hold voltage configuration
    bool setHoldVoltage(float maxVoltage);  //Set the hold voltage of the motor. Voltage is a float from 0 to 3.3V.
    float getHoldVoltage(); //Returns the hold voltage
    bool setRunVoltage(float maxVoltage);   //Set the run voltage of the motor. Voltage is a float from 0 to 3.3V.
    float getRunVoltage();  //Retruns the run voltage

    //Internal I2C Abstraction
    bool read(sfe_qs_register reg, uint8_t *buff, uint8_t buffSize);    //I2C read function for multiple bytes
    bool read(sfe_qs_register reg, uint8_t data);   //I2C read function for one byte
    bool write(sfe_qs_register reg, uint8_t *buff, uint8_t buffSize);   //I2C write function for multiple bytes
    bool write(sfe_qs_register reg, uint8_t data);  //I2C write function for one byte
};
#endif