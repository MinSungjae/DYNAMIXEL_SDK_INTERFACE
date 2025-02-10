#ifndef __DYNAMIXEL_SDK_ADDRESSES_HPP__
#define __DYNAMIXEL_SDK_ADDRESSES_HPP__

enum class DXL_TORQUE : unsigned char
{
    TORQUE_OFF              = 0,
    TORQUE_ON               = 1
};

enum class DXL_LED : unsigned char
{
    LED_OFF                 = 0,
    LED_ON                  = 1
};

enum class DXL_OPERATING_MODE : unsigned char
{
    CURRENT_CONTROL_MODE    = 0,
    VELOCITY_CONTROL_MODE   = 1,
    POSITION_CONTROL_MODE   = 3,
    EXTENDED_POSITION_MODE  = 4,
    CURRENT_POSITION_MODE   = 5,
    PWM_CONTROL_MODE        = 16,
};

#endif