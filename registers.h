//Register Pointer Map
enum Qwiic_Step_Register : uint8_t
{
    QS_ID = 0x00,
    QS_FIRMWARE = 0x01,
    QS_INTERRUPT_CONFIG = 0x03,
    QS_STATUS = 0x04,
    QS_DEVICE_CONFIG = 0x05,
    QS_MOTOR_CONTROL = 0x06,
    QS_CURRENT_POSITION = 0x07,
    QS_DIST_TO_GO = 0x0B,
    QS_MOVE = 0x0F,
    QS_ENABLE_MOVE_NVM = 0x13,
    QS_MOVE_TO = 0x14,
    QS_MAX_SPEED = 0x18,
    QS_ACCELERATION = 0x1C,
    QS_SPEED = 0x20,
    QS_ENABLE_SPEED_NVM = 0x24,
    QS_HOLD_CURRENT = 0x25,
    QS_RUN_CURRENT = 0x27,
    QS_I2C_ADDRESS = 0x29,
};

typedef union {
    struct
    {
        bool isReachedInterruptEnable : 1;
        bool isLimitedInterruptEnable : 1;
        bool : 6;
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
        bool isEStopped : 1;
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
        bool disableMotorOnPositionReached : 1; //If 1, stepper is disabled once requested position is reached.
        bool stopOnLimitSwitchPress : 1;        //If 1, motor will stop when limit switch is pressed.
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
        bool hardStop : 1;
        bool disableMotor : 1;
        bool : 3;
    };
    uint8_t byteWrapped;
} motorControlBitField;