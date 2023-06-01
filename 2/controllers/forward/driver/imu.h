#ifndef __IMU_H__
#define __IMU_H__

#include <webots/robot.h>
#include <webots/inertial_unit.h>


typedef struct
{
	double roll;
	double pitch;
	double yaw;
}
imu_t;

extern imu_t imu;


void imu_init(int time_step);

void imu_update(void);


#endif



