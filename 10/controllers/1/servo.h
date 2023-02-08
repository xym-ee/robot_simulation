#ifndef __SERVO_H__
#define __SERVO_H__

#include <webots/robot.h>
#include <webots/motor.h>

void servo_init(void);

void servo_set_position(double lf, double lb, double rf, double rb);

#endif // ! __SERVO_H__
