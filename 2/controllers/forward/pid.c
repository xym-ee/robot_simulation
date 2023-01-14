#include "pid.h"


double pid_control(pid_t *pid, double ref, double feedback)
{
    pid->feedback = feedback;
    pid->ref = ref;

    pid->err = pid->ref - pid->feedback;
    pid->err_sum += pid->err;
    pid->output = pid->kp * pid->err + \
                  pid->ki * pid->err_sum + \
                  pid->kd * (pid->err - pid->err1);

    pid->err1 = pid->err;

    if (pid->out_lim > 1e-8)
    {
        if (pid->output > pid->out_lim)
            pid->output = pid->out_lim;
        if (pid->output < -pid->out_lim)
            pid->output = -pid->out_lim;
    }

    return pid->output;
}

void pid_set_parameter(pid_t* pid, double kp, double ki, double kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->ref = 0;
    pid->feedback = 0;
    pid->err_sum = 0.0;
    pid->output = 0;
}

void pid_set_output_limit(pid_t* pid, double limit)
{
    pid->out_lim = limit;
}




