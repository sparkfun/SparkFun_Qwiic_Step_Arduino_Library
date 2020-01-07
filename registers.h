//Register Pointer Map
enum Qwiic_Step_Register : uint8_t
{
    ID = 0x00,
    FIRMWARE = 0x01,
    INTERRUPT_CONFIG = 0x03,
    STATUS = 0x04,
    DEVICE_CONFIG = 0x05,
    DEVICE_CONTROL = 0x06,
    CURRENT_POSITION = 0x07,
    DIST_TO_GO = 0x0B,
    MOVE = 0x0F,
    ENABLE_MOVE_NVM = 0x13,
    MOVE_TO = 0x14,
    MAX_SPEED = 0x18,
    ACCELERATION = 0x1C,
    SPEED = 0x20,
    ENABLE_SPEED_NVM = 0x24,
    HOLD_CURRENT = 0x25,
    RUN_CURRENT = 0x27,
    I2C_ADDRESS = 0x29,
};

typedef union {
    struct
    {
        bool requestedPosReachedEnable : 1;
        bool requestedPosReachedIntTriggered : 1;
        bool limSwitchPressedEnable : 1;
        bool limSwitchPressedIntTriggered : 1;
        bool : 4;
    };
    uint8_t byteWrapped;
} interruptConfigBitField;

typedef union {
    struct
    {
        bool isRunning : 1;      //This is bit 0. Gets set to 1 when motor is turning, 0 if not.
        bool isAccelerating : 1; //Gets set to 1 when motor is accelerating, 0 otherwise.
        bool isDecelerating : 1; //Gets set to 1 when motor is decelerating, 0 otherwise.
        bool isReached : 1;
        bool isLimited : 1;
        bool eStopped : 1;
        bool : 2;
    };
    uint8_t byteWrapped;
} statusRegisterBitField;

typedef union {
    struct
    {
        bool MS1 : 1; //This starts at bit 0. Bits indicate micro step mode (MS3 MS2 MS1).
        bool MS2 : 1;
        bool MS3 : 1;
        bool disableMotorPositionReached : 1; //User-mutable. If 1, stepper is disabled once requested position is reached.
        bool stopOnLimitSwitchPress : 1;      //User-mutable. If 1, motor will stop when limit switch is pressed.
        bool : 3;
    };
    uint8_t byteWrapped;
} deviceConfigBitField;

typedef union {
    struct
    {
        bool run : 1;
        bool runSpeed : 1;
        bool runSpeedToPosition : 1;
        bool stop : 1;
        bool disableMotor : 1;
        bool : 3;
    };
    uint8_t byteWrapped;
} deviceControlBitField;