#include <webots/robot.h>
#include <webots/motor.h>

#include "servo.h"

#define TIME_STEP 64


int main(int argc, char** argv)
{
    wb_robot_init();

    servo_init();

    servo_set_position(180,180, 180, 180);

    WbDeviceTag motor_l = wb_robot_get_device("motor_l");
    WbDeviceTag motor_r = wb_robot_get_device("motor_r");


    while (wb_robot_step(TIME_STEP) != -1)
    {

    };

    wb_robot_cleanup();

    return 0;
}
