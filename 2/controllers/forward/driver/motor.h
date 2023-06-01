#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <webots/robot.h>
#include <webots/motor.h>


#include "../rtdef.h"


/* 电机信息 */
typedef struct
{
	/* 电机角度 */
	rt_int64_t angle;

	/* 转矩电流 */
	rt_int16_t current;

	rt_int16_t speed;

	rt_uint16_t encoder;

} motor_sensot_t;

typedef enum
{
	MODE_TORQUE = 1,
	MODE_SPEED,
	MODE_POSITION,
}
motor_control_mode_t;

/* 电机控制给定值 */
typedef struct
{
	/* 电机控制模式 */
	motor_control_mode_t mode;

	/* 电流给定值 -2000 ~ 2000 */
	rt_int16_t torque;

	/* 速度给定值 单位 0.01°/s */
	rt_int32_t speed;

	rt_int32_t position;

} motor_control_t;

/* 电机信息 */
typedef struct
{
	motor_sensot_t sensor;
	motor_control_t control;
} motor_single_t;



typedef struct
{
	motor_single_t left;
	motor_single_t right;

} motor_t;


extern motor_t motor;

void motor_init(void);

#endif



