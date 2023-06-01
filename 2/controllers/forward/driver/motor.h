#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <webots/robot.h>
#include <webots/motor.h>


#include "../rtdef.h"


/* �����Ϣ */
typedef struct
{
	/* ����Ƕ� */
	rt_int64_t angle;

	/* ת�ص��� */
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

/* ������Ƹ���ֵ */
typedef struct
{
	/* �������ģʽ */
	motor_control_mode_t mode;

	/* ��������ֵ -2000 ~ 2000 */
	rt_int16_t torque;

	/* �ٶȸ���ֵ ��λ 0.01��/s */
	rt_int32_t speed;

	rt_int32_t position;

} motor_control_t;

/* �����Ϣ */
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



