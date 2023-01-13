#include "motor.h"


WbDeviceTag motor_left;
WbDeviceTag motor_right;


void motor_init(void)
{
    motor_left = wb_robot_get_device("motor_left");
    motor_right = wb_robot_get_device("motor_right");

}





