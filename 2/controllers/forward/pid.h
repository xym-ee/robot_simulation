#ifndef __PID_H__
#define __PID_H__




struct _pid_t
{
    float kp;
    float ki;
    float kd;

    float err;
    float err1;
    float err2;


    float output;
    float out_lim;

};


typedef struct _pid_t* pid_t;



float pid_controller(pid_t p);




#endif // !__PID_H__




