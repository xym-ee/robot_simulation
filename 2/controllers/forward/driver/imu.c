#include "imu.h"

imu_t imu;

WbDeviceTag imu_tag;


void imu_init(int time_step)
{
    imu_tag = wb_robot_get_device("imu");

    wb_inertial_unit_enable(imu_tag, time_step);
}


void imu_update(void)
{
    const double* imu_buf = NULL;

    imu_buf = wb_inertial_unit_get_roll_pitch_yaw(imu_tag);

    imu.roll = imu_buf[0];
    imu.pitch = imu_buf[1];
    imu.yaw = imu_buf[2];

    imu.roll = imu.roll * 180.0 / 3.141593;
    imu.pitch = imu.pitch * 180.0 / 3.141593;
    imu.yaw = imu.yaw * 180.0 / 3.141593;
}











