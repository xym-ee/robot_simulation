#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/inertial_unit.h>
#include <webots/position_sensor.h>
#include <webots/keyboard.h>
#include <math.h>
#include <stdio.h>
#include <malloc.h>

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
    double p_ref = 40, p_feedback = 0;
    double p_err = 0, p_err1 = 0;
    double p_out = 0;

    /* velocity pd controller */
    double velocity_ref = 0, velocity_feedback = 0;
    double v_err = 0, v_err1 = 0;
    double v_out = 0;

    /* angle pd controller */
    double angle_ref = 0, angle_feedback = 0;
    double err = 0, err1 = 0;
    double motor_torque = 0;

    while (wb_robot_step(TIME_STEP) != -1)
    {
        /* encoder data update */
        position_left = wb_position_sensor_get_value(encoder_left);
        position_right = wb_position_sensor_get_value(encoder_right);

        position = (position_left + position_right) / 2;

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

        //        printf("roll %f pitch %f yaw %f \n", roll, pitch, yaw);

                /* position pd controller.  */
        p_feedback = -position;
        p_err = p_ref - p_feedback;
        p_out = 0.02 * p_err + 0.1 * (p_err - p_err1);
        p_err1 = p_err;

        if (p_out > 0.2)
            p_out = 0.2;
        if (p_out < -0.2)
            p_out = -0.2;

        velocity_ref = p_out;

        /* velocity pd controller.  */
        velocity_feedback = -velocity;
        v_err = velocity_ref - velocity_feedback;
        v_out = 62.0 * v_err + 0.0 * (v_err - v_err1);
        v_err1 = v_err;

        if (v_out > 10.0)
            v_out = 10.0;
        if (v_out < -10.0)
            v_out = -10.0;

        angle_ref = v_out;

        /* angle pd controller.  */
        _pid_t _angle_controller;
        pid_t angle_controller = &_angle_controller;
        pid_controller_init(angle_controller, 0.008, 0, 0.08);



        angle_controller->ref = 0;
        angle_controller->feedback = -pitch;

        angle_feedback = -pitch;
        err = angle_ref - angle_feedback;
        motor_torque = 0.008 * err1 + 0.08 * (err - err1);
        err1 = err;

        wb_motor_set_torque(motor_left, motor_torque);
        wb_motor_set_torque(motor_right, motor_torque);

    };

    wb_robot_cleanup();

    return 0;
}





