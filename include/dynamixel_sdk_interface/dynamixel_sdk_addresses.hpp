#ifndef __DYNAMIXEL_SDK_ADDRESSES_HPP__
#define __DYNAMIXEL_SDK_ADDRESSES_HPP__

#define ADDR_XX_DRIVE_MODE              10
#define ADDR_XX_OPERATING_MODE          11
#define ADDR_XX_CURRENT_LIMIT           38

#define ADDR_XX_TORQUE_ENABLE           64
#define ADDR_XX_LED                     65

#define ADDR_XX_POSITION_P_GAIN         84
#define ADDR_XX_POSITION_I_GAIN         82
#define ADDR_XX_POSITION_D_GAIN         80
#define ADDR_XX_VELOCITY_P_GAIN         78
#define ADDR_XX_VELOCITY_I_GAIN         76
#define ADDR_XX_FF_VELOCITY_GAIN        90
#define ADDR_XX_FF_ACCELERATION_GAIN    88

#define ADDR_XX_GOAL_POSITION           116
#define ADDR_XX_GOAL_VELOCITY           104
#define ADDR_XX_GOAL_CURRENT            102

#define ADDR_XX_PROFILE_VELOCITY        112

#define ADDR_XX_PRESENT_POSITION        132
#define ADDR_XX_PRESENT_VELOCITY        128
#define ADDR_XX_PRESENT_CURRENT         126
#define ADDR_XX_PRESENT_PWM             124

#define ADDR_XX_PRESENT_TEMPERATURE     146

#define SIZE_XX_PRESENT_POSITION        4
#define SIZE_XX_PRESENT_VELOCITY        4
#define SIZE_XX_GOAL_POSITION           4
#define SIZE_XX_PRESENT_TEMPERATURE     1
#define SIZE_XX_PROFILE_VELOCITY        4
#define SIZE_XX_CURRENT_LIMIT           2
#define SIZE_XX_GOAL_CURRENT            2

enum class DXL_TORQUE : uint8_t
{
    TORQUE_OFF              = 0,
    TORQUE_ON               = 1
};

enum class DXL_LED : uint8_t
{
    LED_OFF                 = 0,
    LED_ON                  = 1
};

enum class DXL_OPERATING_MODE : uint8_t
{
    CURRENT_CONTROL_MODE    = 0,
    VELOCITY_CONTROL_MODE   = 1,
    POSITION_CONTROL_MODE   = 3,
    EXTENDED_POSITION_MODE  = 4,
    CURRENT_POSITION_MODE   = 5,
    PWM_CONTROL_MODE        = 16,
};

#endif