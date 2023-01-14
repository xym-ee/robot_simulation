#ifndef __PID_H__
#define __PID_H__

#include <math.h>

typedef struct 
{
    double kp;
    double ki;
    double kd;

    double ref;
    double feedback;

    double err;
    double err1;

    double output;
    double output_lim;
} _pid_t;

typedef _pid_t* pid_t;

void pid_controller_init(pid_t* p, double kp, double ki, double kd);







#endif // !__PID_H__

