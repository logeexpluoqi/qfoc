/*
 * @Author: luoqi
 * @Date: 2021-04-27 19:20:38
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-01-26 17:05:17
 */

#include "pid.h"

int pid_init(PidObj *pid, float kp, float ki, float kd, float olimit)
{
    pid->delta_yk = 0.0f;
    pid->err_k2 = 0.0f;
    pid->err_k1 = 0.0f;
    pid->yk1 = 0.0f;
    pid->yk = 0.0f;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->olimit = olimit;
    return 0;
}

int pid_param_set(PidObj *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    return 0;
}

float pid_calc(PidObj *pid, float err)
{
    pid->delta_yk = pid->kp * (err - pid->err_k1) +
                     pid->ki * err +
                     pid->kd * (err - 2 * pid->err_k1 + pid->err_k2);

    pid->err_k2 = pid->err_k1;
    pid->err_k1 = err;
    pid->yk = pid->yk1 + pid->delta_yk;

    if(pid->olimit != 0){
        if (pid->yk > pid->olimit) {
            pid->yk = pid->olimit;
            pid->yk1 = pid->yk;
        } else if (pid->yk < -pid->olimit) {
            pid->yk = -pid->olimit;
            pid->yk1 = pid->yk;
        } else {
            pid->yk1 = pid->yk;
        }
    }
    return pid->yk;
}

int pid_clear(PidObj *pid)
{
    pid->yk = 0;
    pid->yk1 = 0;
    pid->delta_yk = 0;
    pid->err_k1 = 0;
    pid->err_k2 = 0;
    return 0;
}

int pid_calc_clear(PidObj *pid)
{
    pid->delta_yk = 0;
    pid->err_k1 = 0;
    pid->err_k2 = 0;
    return 0;
}

float pid_ki_isolate_calcu(PidObj *pid, float err, float seplimit)
{
    int beta = err > seplimit ? 0 : 1;

    pid->delta_yk = pid->kp * (err - pid->err_k1) +
                     (float) beta * pid->ki * err +
                     pid->kd * (err - 2 * pid->err_k1 + pid->err_k2);

    pid->err_k2 = pid->err_k1;
    pid->err_k1 = err;
    pid->yk = pid->yk1 + pid->delta_yk;

    if(pid->olimit != 0){
        if (pid->yk > pid->olimit) {
            pid->yk = pid->olimit;
            pid->yk1 = pid->yk;
        } else if (pid->yk < -pid->olimit) {
            pid->yk = -pid->olimit;
            pid->yk1 = pid->yk;
        } else {
            pid->yk1 = pid->yk;
        }
    }
    return pid->yk;
}
