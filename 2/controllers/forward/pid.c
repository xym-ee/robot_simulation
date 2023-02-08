#include "pid.h"


double controller_output(controller_t *controller, double ref, double feedback)
{
    controller->feedback = feedback;
    controller->ref = ref;

    controller->err = controller->ref - controller->feedback;

    /* ���ַ��룬�����󲻻��� */
    if (controller->err > controller->int_boundary || controller->err < -controller->int_boundary)
    {
        controller->err_sum = 0;
    }
    else
    {
        controller->err_sum += controller->err;
    }

    controller->output = controller->kp * controller->err + \
                         controller->ki * controller->err_sum + \
                         controller->kd * (controller->err - controller->err1);

    controller->err1 = controller->err;

    /* ����޷� */
    if (controller->out_lim > 1e-8)
    {
        if (controller->output > controller->out_lim)
            controller->output = controller->out_lim;
        if (controller->output < -controller->out_lim)
            controller->output = -controller->out_lim;
    }

    return controller->output;
}

void controller_set_pid_parameter(controller_t *controller, double kp, double ki, double kd)
{
    controller->kp = kp;
    controller->ki = ki;
    controller->kd = kd;

    controller->ref = 0.0;
    controller->feedback = 0.0;
    controller->output = 0.0;

    controller->err = 0.0;
    controller->err1 = 0.0;

    controller->err_sum = 0.0;
}

void controller_set_output_limit(controller_t *controller, double limit)
{
    controller->out_lim = limit;
}




