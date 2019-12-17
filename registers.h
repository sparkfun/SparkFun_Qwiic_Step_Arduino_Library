//Register Pointer Map
enum Qwiic_Step_Register : uint8_t
{
    ID = 0x00,
    FIRMWARE = 0x01,
    INTERRUPT_ENABLE = 0x03,
    STATUS = 0x04,
    DEVICE_CONFIG = 0x05,
    DEVICE_CONTROL = 0x06,
    CURRENT_POSITION = 0x07,
    DIST_TO_GO = 0x0B,
    MOVE = 0x0F,
    MOVE_TO = 0x13,
    MAX_SPEED = 0x17,
    ACCELERATION = 0x1B,
    SET_SPEED = 0x1F,
    ENABLE_SET_SPEED = 0x23,
    HOLD_CURRENT = 0x24,
    RUN_CURRENT = 0x26,
    I2C_ADDRESS = 0x28,
};

typedef union {
    struct
    {
        bool requestedPosReachedEnable : 1;
        bool limSwitchPressedEnable : 1;
        bool : 6;
    };
    uint8_t byteWrapped;
} interruptEnableBitField;

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
        bool powerDownPositionReached : 1; //User-mutable. If 1, stepper is disabled once requested position is reached.
        bool stopOnLimitSwitchPress : 1;   //User-mutable. If 1, motor will stop when limit switch is pressed.
        bool : 3;
    };
    uint8_t byteWrapped;
} deviceConfigBitField;

typedef union {
    struct
    {
        bool stop : 1; //Not really sure about these yettttt
        bool runTo : 1;
        bool runContinuous : 1;
        bool sleep : 1;
        bool : 4;
    };
    uint8_t byteWrapped;
} deviceControlBitField;