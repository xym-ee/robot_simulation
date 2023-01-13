#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>
#include <math.h>
#include <stdio.h>

#include "pid.h"
#include "motor.h"

#define TIME_STEP 8

#define MAX_SPEED 6.28

#define PI          3.141593f

int main(int argc, char** argv)
{
    wb_robot_init();

    motor_init();


    /*
    wb_motor_set_position(motor_left, INFINITY);
    wb_motor_set_position(motor_right, INFINITY);

    wb_motor_set_velocity(motor_left, 0 * MAX_SPEED);
    wb_motor_set_velocity(motor_right, 0 * MAX_SPEED);
    */

    WbDeviceTag encoder_left = wb_robot_get_device("encoder_left");
    WbDeviceTag encoder_right = wb_robot_get_device("encoder_right");

    wb_position_sensor_enable(encoder_left, TIME_STEP);
    wb_position_sensor_enable(encoder_right, TIME_STEP);

    WbDeviceTag imu = wb_robot_get_device("imu");

    wb_inertial_unit_enable(imu, TIME_STEP);

    printf("init successed ...\n");

    const double* imu_buf = NULL;
    double roll = 0, pitch = 0, yaw = 0;

    double err = 0, err1 = 0;
    double motor_torque = 0;

    double velocity = 0, left_velocity = 0, right_velocity = 0;


    while (wb_robot_step(TIME_STEP) != -1)
    {
        left_velocity = wb_position_sensor_get_value(encoder_left);
        right_velocity = wb_position_sensor_get_value(encoder_right);

        printf("%f %f \n", left_velocity, right_velocity);


        /* imu ���ݸ��� */
        imu_buf = wb_inertial_unit_get_roll_pitch_yaw(imu);

        roll  = imu_buf[0];
        pitch = imu_buf[1];
        yaw   = imu_buf[2];

        roll = roll * 180.0 / PI;
        pitch = pitch * 180.0 / PI;
        yaw = yaw * 180.0 / PI;

        err1 = err;
        err = pitch;

        motor_torque = 0.008 * err1 + 0.08 * (err - err1);

        wb_motor_set_torque(motor_left, motor_torque);
        wb_motor_set_torque(motor_right, motor_torque);


        printf("roll %f pitch %f yaw %f \n", roll, pitch, yaw);
    };

    wb_robot_cleanup();

    return 0;
}





