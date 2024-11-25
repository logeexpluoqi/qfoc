/*
 * @Author: luoqi
 * @Date: 2021-04-27 19:20:38
 * @ Modified by: luoqi
 * @ Modified time: 2024-11-25 17:51
 */

#include "pid.h"

static inline float _abs(float x)
{
    return x > 0 ? x : -x;
}

int pid_init(PidObj *pid, float kp, float ki, float kd, float olimit)
{
    pid->delta_k = 0;
    pid->ek1 = 0;
    pid->edk1 = 0;
    pid->edk2 = 0;
    pid->yk1 = 0;
    pid->yk = 0;
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
    float edk = err - pid->ek1;
    pid->delta_k = pid->kp * (err - pid->ek1)
        + pid->ki * err
        + pid->kd * (edk - pid->edk1);

    pid->edk1 = edk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    if(pid->olimit != 0) {
        if(pid->yk > pid->olimit) {
            pid->yk = pid->olimit;
            pid->yk1 = pid->yk;
        } else if(pid->yk < -pid->olimit) {
            pid->yk = -pid->olimit;
            pid->yk1 = pid->yk;
        } else {
            pid->yk1 = pid->yk;
        }
    }
    return pid->yk;
}

int pid_incplt_diff_init(PidObj *pid, float kp, float ki, float kd, float alpha, float olimit)
{
    pid_init(pid, kp, ki, kd, olimit);
    if((alpha < 0) || (alpha > 1)) {
        pid->alpha = 1;
        return -1;
    }
    pid->alpha = alpha;
    return 0;
}

float pid_incplt_diff_calc(PidObj *pid, float err)
{
    float edk = err - pid->ek1;

    pid->delta_k = pid->kp * (err - pid->ek1)
        + pid->ki * err
        + pid->kd * (pid->alpha * (edk - 2 * pid->edk1 + pid->edk2) + pid->edk1 - pid->edk2);
    pid->edk2 = pid->edk1;
    pid->edk1 = edk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    if(pid->olimit != 0) {
        if(pid->yk > pid->olimit) {
            pid->yk = pid->olimit;
            pid->yk1 = pid->yk;
        } else if(pid->yk < -pid->olimit) {
            pid->yk = -pid->olimit;
            pid->yk1 = pid->yk;
        } else {
            pid->yk1 = pid->yk;
        }
    }
    return pid->yk;
}

/* integral varible PID */
int pid_int_var_init(PidObj *pid, float kp, float ki, float kd, float lth, float hth, float olimit)
{
    pid_init(pid, kp, ki, kd, olimit);

    if((lth < 0) || (hth < 0) || (lth > hth)) {
        pid->alpha = 1;
        return -1;
    }

    pid->alpha = lth;
    pid->hth = hth;
    return 0;
}

float pid_int_var_calc(PidObj *pid, float err)
{
    float ratio = 0;

    if(err <= pid->alpha ) { // e < lth
        ratio = 1;
    } else if(err > pid->hth) { // e > hth
        ratio = 0;
    } else { // lth < e < hth
        ratio = (pid->hth - _abs(err)) / pid->alpha;
    }

    float edk = err - pid->ek1;

    pid->delta_k = pid->kp * (err - pid->ek1)
        + ratio * pid->ki * err
        + pid->kd * (edk - pid->edk1);

    pid->edk1 = edk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    if(pid->olimit != 0) {
        if(pid->yk > pid->olimit) {
            pid->yk = pid->olimit;
            pid->yk1 = pid->yk;
        } else if(pid->yk < -pid->olimit) {
            pid->yk = -pid->olimit;
            pid->yk1 = pid->yk;
        } else {
            pid->yk1 = pid->yk;
        }
    }
    return pid->yk;
}

int pid_diff_first_init(PidObj *pid, float kp, float ki, float kd, float alpha, float olimit)
{
    pid_init(pid, kp, ki, kd, olimit);
    if(pid->alpha < 0 || pid->alpha > 1) {
        pid->alpha = 0;
        return -1;
    }
    pid->alpha = alpha;
    return 0;
}

float pid_diff_first_calc(PidObj *pid, float err)
{
    float eyk = pid->yk - pid->yk1;

    pid->delta_k = pid->kp * (err - pid->ek1)
        + pid->ki * err
        + pid->kd * (pid->alpha * (eyk - 2 * pid->eyk1 + pid->eyk2) + pid->eyk1 - pid->eyk2); 

    pid->eyk2 = pid->eyk1;
    pid->eyk1 = eyk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    if(pid->olimit != 0) {
        if(pid->yk > pid->olimit) {
            pid->yk = pid->olimit;
            pid->yk1 = pid->yk;
        } else if(pid->yk < -pid->olimit) {
            pid->yk = -pid->olimit;
            pid->yk1 = pid->yk;
        } else {
            pid->yk1 = pid->yk;
        }
    }
    return pid->yk;
}

int pid_int_sep_init(PidObj *pid, float kp, float ki, float kd, float alpha, float olimit)
{
    pid_init(pid, kp, ki, kd, olimit);
    if(pid->alpha < 0) {
        pid->alpha = 1;
        return -1;
    }
    pid->alpha = alpha;
    return 0;
}

float pid_int_sep_calc(PidObj *pid, float err)
{
    int th = _abs(err) > pid->alpha ? 0 : 1;
    float edk = err - pid->ek1;

    pid->delta_k = pid->kp * (err - pid->ek1)
        + th * pid->ki * err
        + pid->kd * (edk - pid->edk1);

    pid->edk1 = edk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    if(pid->olimit != 0) {
        if(pid->yk > pid->olimit) {
            pid->yk = pid->olimit;
            pid->yk1 = pid->yk;
        } else if(pid->yk < -pid->olimit) {
            pid->yk = -pid->olimit;
            pid->yk1 = pid->yk;
        } else {
            pid->yk1 = pid->yk;
        }
    }
    return pid->yk;
}

int pid_clr(PidObj *pid)
{
    pid->yk = 0;
    pid->yk1 = 0;
    pid->delta_k = 0;
    pid->ek1 = 0;
    pid->edk1 = 0;
    pid->edk2 = 0;
    pid->eyk2 = 0;
    return 0;
}

int pid_calc_clr(PidObj *pid)
{
    pid->delta_k = 0;
    pid->ek1 = 0;
    pid->edk1 = 0;
    pid->edk2 = 0;
    pid->eyk2 = 0;
    return 0;
}
