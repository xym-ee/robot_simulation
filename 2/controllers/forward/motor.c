#include "motor.h"


WbDeviceTag motor_left;
WbDeviceTag motor_right;


void motor_init(void)
{
    motor_left  = wb_robot_get_device("motor_left");
    motor_right = wb_robot_get_device("motor_right");
}

void motor_torque_control_left(double t)
{
    wb_motor_set_torque(motor_left, t);
}

void motor_torque_control_right(double t)
{
    wb_motor_set_torque(motor_right, t);
}



