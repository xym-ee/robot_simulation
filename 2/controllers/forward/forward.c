#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>
#include <math.h>
#include <stdio.h>

#include "pid.h"

#define TIME_STEP   8

#define MAX_SPEED   6.28

#define PI          3.141593f

int main(int argc, char** argv)
{
    wb_robot_init();

    /*---------------------- motor --------------------*/
    WbDeviceTag motor_left = wb_robot_get_device("motor_left");
    WbDeviceTag motor_right = wb_robot_get_device("motor_right");

    double motor_torque = 0.0;

    /*---------------------- encoder --------------------*/
    WbDeviceTag encoder_left = wb_robot_get_device("encoder_left");
    WbDeviceTag encoder_right = wb_robot_get_device("encoder_right");

    wb_position_sensor_enable(encoder_left, TIME_STEP);
    wb_position_sensor_enable(encoder_right, TIME_STEP);

    double position = 0;
    double position_left = 0, position_left1 = 0;
    double position_right = 0, position_right1 = 0;
    double velocity = 0, velocity_left = 0, velocity_right = 0;

    /*---------------------- IMU --------------------*/
    WbDeviceTag imu = wb_robot_get_device("imu");

    wb_inertial_unit_enable(imu, TIME_STEP);

    const double* imu_buf = NULL;
    double roll = 0, pitch = 0, yaw = 0;

    printf("init successed ...\n");

    /* position pd controller */
    double position_ref = 40.0;
    double position_feedback = 0.0;
    controller_t position_controller;

    controller_set_pid_parameter(&position_controller, 0.011, 0.0, 0.03);
    controller_set_output_limit(&position_controller, 0.2);

    /* velocity pd controller */
    double velocity_ref = 0.0;
    double velocity_feedback = 0.0;
    controller_t velocity_controller;

    controller_set_pid_parameter(&velocity_controller, 65.0, 0.0, 20.0);
    controller_set_output_limit(&velocity_controller, 10.0);

    /* angle pd controller */
    double angle_ref = 0.0;
    double angle_feedback = 0.0;
    controller_t angle_controller;

    controller_set_pid_parameter(&angle_controller, 0.008, 0, 0.05);

    while (wb_robot_step(TIME_STEP) != -1)
    {
        /* encoder data update */
        position_left = wb_position_sensor_get_value(encoder_left);
        position_right = wb_position_sensor_get_value(encoder_right);

        position = (position_left + position_right) / 2;
        printf("%f\n", position);
        velocity_left = position_left - position_left1;
        velocity_right = position_right - position_right1;

        velocity = (velocity_left + velocity_right) / 2;

        position_left1 = position_left;
        position_right1 = position_right;

        printf("%f %f \n", velocity_left, velocity_right);

        /* imu data update */
        imu_buf = wb_inertial_unit_get_roll_pitch_yaw(imu);

        roll = imu_buf[0];
        pitch = imu_buf[1];
        yaw = imu_buf[2];

        roll = roll * 180.0 / PI;
        pitch = pitch * 180.0 / PI;
        yaw = yaw * 180.0 / PI;

        //printf("roll %f pitch %f yaw %f \n", roll, pitch, yaw);

        /* position pd controller.  */
        position_feedback = -position;
        velocity_ref = controller_output(&position_controller, position_ref, position_feedback);

        /* velocity pd controller.  */
        velocity_feedback = -velocity;
        angle_ref = controller_output(&velocity_controller, velocity_ref, velocity_feedback);

        /* angle pd controller.  */
        angle_feedback = -pitch;
        motor_torque = controller_output(&angle_controller, angle_ref, angle_feedback);

        wb_motor_set_torque(motor_left, motor_torque);
        wb_motor_set_torque(motor_right, motor_torque);
    };

    wb_robot_cleanup();
    return 0;
}




