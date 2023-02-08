#include "servo.h"

WbDeviceTag servo_lf, servo_lb, servo_rf, servo_rb;

void servo_init(void)
{
    servo_lf = wb_robot_get_device("servo_lf");
    servo_lb = wb_robot_get_device("servo_lb");
    servo_rf = wb_robot_get_device("servo_rf");
    servo_rb = wb_robot_get_device("servo_rb");
}


void servo_set_position(double lf,double lb, double rf, double rb)
{
    wb_motor_set_position(servo_lf, lf * 3.141593 / 180.0);
    wb_motor_set_position(servo_lb, -lb * 3.141593 / 180.0);
    wb_motor_set_position(servo_rf, rf * 3.141593 / 180.0);
    wb_motor_set_position(servo_rb, -rb * 3.141593 / 180.0);
}


