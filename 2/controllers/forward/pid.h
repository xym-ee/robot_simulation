#ifndef __PID_H__
#define __PID_H__

typedef struct 
{
    double ref;
    double feedback;
    double output;

    double kp;
    double ki;
    double kd;

    double out_lim;

    double err;
    double err_sum;
    double err1;
} pid_t;


double pid_control(pid_t* pid, double ref, double feedback);
void pid_set_parameter(pid_t* pid, double kp, double ki, double kd);
void pid_set_output_limit(pid_t* pid, double limit);


#endif // !__PID_H__




