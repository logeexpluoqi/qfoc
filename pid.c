/*
 * @Author: luoqi
 * @Date: 2021-04-27 19:20:38
 * @ Modified by: luoqi
 * @ Modified time: 2025-02-28 22:58
 */

#include "pid.h"

static inline qfp_t _abs(qfp_t x)
{
    return x > 0 ? x : -x;
}

static inline qfp_t update_pid_output(PidObj *pid)
{
    if(!pid) {
        return 0;
    }
    if(pid->olimit != PID_NONE) {
        if(pid->yk > pid->olimit) {
            pid->yk = pid->olimit;
        } else if(pid->yk < -pid->olimit) {
            pid->yk = -pid->olimit;
        }
    }
    pid->yk1 = pid->yk;
    return pid->yk;
}

static inline qfp_t _int_ratio_calc(qfp_t err, qfp_t lth, qfp_t hth)
{
    qfp_t abs_err = _abs(err);
    if(abs_err <= lth) {
        return 1;
    } else if(abs_err > hth) {
        return 0;
    }
    return (hth - abs_err) / (hth - lth);
}

int pid_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    pid->delta_k = 0;
    pid->ek1 = 0;
    pid->edk1 = 0;
    pid->edk2 = 0;
    pid->yk1 = 0;
    pid->yk = 0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->alpha = 1;
    pid->olimit = olimit;
    return 0;
}

int pid_param_set(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd)
{
    if(!pid) {
        return -1;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    return 0;
}

qfp_t pid_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }
    qfp_t edk = 0;
    if(dt != PID_NONE) {
        edk = (err - pid->ek1) / dt;
    } else {
        edk = err - pid->ek1;
    }
    pid->delta_k = pid->kp * (err - pid->ek1)
        + pid->ki * err
        + pid->kd * (edk - pid->edk1);

    pid->edk1 = edk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    return update_pid_output(pid);
}

int pid_int_sep_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    pid_init(pid, kp, ki, kd, olimit);
    if(pid->alpha < 0 || pid->alpha > 1) {
        pid->alpha = 1;
        return -1;
    }
    pid->alpha = alpha;
    return 0;
}

qfp_t pid_int_sep_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    int th = _abs(err) > pid->alpha ? 0 : 1;
    qfp_t edk = 0;
    if(dt != PID_NONE) {
        edk = (err - pid->ek1) / dt;
    } else {
        edk = err - pid->ek1;
    }

    pid->delta_k = pid->kp * (err - pid->ek1)
        + th * pid->ki * err
        + pid->kd * (edk - pid->edk1);

    pid->edk1 = edk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    return update_pid_output(pid);
}

int pid_incplt_diff_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    
    pid_init(pid, kp, ki, kd, olimit);
    if((alpha < 0) || (alpha > 1)) {
        pid->alpha = 1;
        return -1;
    }
    pid->alpha = alpha;
    return 0;
}

qfp_t pid_incplt_diff_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    qfp_t edk = 0;
    if(dt != PID_NONE) {
        edk = (err - pid->ek1) / dt;
    } else {
        edk = err - pid->ek1;
    }

    pid->delta_k = pid->kp * (err - pid->ek1)
        + pid->ki * err
        + pid->kd * (pid->alpha * (edk - 2 * pid->edk1 + pid->edk2) + pid->edk1 - pid->edk2);
    pid->edk2 = pid->edk1;
    pid->edk1 = edk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    return update_pid_output(pid);
}

/* integral varible PID */
int pid_int_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t lth, qfp_t hth, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    
    pid_init(pid, kp, ki, kd, olimit);

    if((lth < 0) || (hth < 0) || (lth > hth)) {
        pid->lth = 1;
        return -1;
    }

    pid->lth = lth;
    pid->hth = hth;
    return 0;
}

qfp_t pid_int_var_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    qfp_t ratio = 0;
    qfp_t edk = 0;
    if(dt != PID_NONE) {
        edk = (err - pid->ek1) / dt;
    } else {
        edk = err - pid->ek1;
    }

    ratio = _int_ratio_calc(err, pid->lth, pid->hth);

    pid->delta_k = pid->kp * (err - pid->ek1)
        + ratio * pid->ki * err
        + pid->kd * (edk - pid->edk1);

    pid->edk1 = edk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    return update_pid_output(pid);
}

int pid_diff_first_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    
    pid_init(pid, kp, ki, kd, olimit);
    if(pid->alpha < 0 || pid->alpha > 1) {
        pid->alpha = 0;
        return -1;
    }
    pid->alpha = alpha;
    return 0;
}

qfp_t pid_diff_first_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    qfp_t eyk = pid->delta_k;

    pid->delta_k = pid->kp * (err - pid->ek1)
        + pid->ki * err
        + pid->kd * (pid->alpha * (eyk - 2 * pid->eyk1 + pid->eyk2) + pid->eyk1 - pid->eyk2);

    pid->eyk2 = pid->eyk1;
    pid->eyk1 = eyk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    return update_pid_output(pid);
}

/* incomplete differential and integral varible PID */
int pid_incplt_diff_int_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t lth, qfp_t hth, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    
    pid_init(pid, kp, ki, kd, olimit);
    if((alpha < 0) || (alpha > 1)) {
        pid->alpha = 1;
        return -1;
    }
    pid->alpha = alpha;
    if((lth < 0) || (hth < 0) || (lth > hth)) {
        pid->lth = 1;
        return -1;
    }
    pid->lth = lth;
    pid->hth = hth;
    return 0;
}

qfp_t pid_incplt_diff_int_var_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    qfp_t edk = 0;
    if(dt != PID_NONE) {
        edk = (err - pid->ek1) / dt;
    } else {
        edk = err - pid->ek1;
    }
    qfp_t ratio = _int_ratio_calc(err, pid->lth, pid->hth);

    pid->delta_k = pid->kp * (err - pid->ek1)
        + ratio * pid->ki * err
        + pid->kd * (pid->alpha * (edk - 2 * pid->edk1 + pid->edk2) + pid->edk1 - pid->edk2);
    pid->edk2 = pid->edk1;
    pid->edk1 = edk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    return update_pid_output(pid);
}

/* differential first and integral varible PID */
int pid_diff_first_int_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t lth, qfp_t hth, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    
    pid_init(pid, kp, ki, kd, olimit);
    if((alpha < 0) || (alpha > 1)) {
        pid->alpha = 1;
        return -1;
    }
    pid->alpha = alpha;
    if((lth < 0) || (hth < 0) || (lth > hth)) {
        pid->lth = 1;
        return -1;
    }
    pid->lth = lth;
    pid->hth = hth;
    return 0;
}

qfp_t pid_diff_first_int_var_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    qfp_t eyk = pid->delta_k;
    qfp_t ratio = _int_ratio_calc(err, pid->lth, pid->hth);

    pid->delta_k = pid->kp * (err - pid->ek1)
        + ratio * pid->ki * err
        + pid->kd * (pid->alpha * (eyk - 2 * pid->eyk1 + pid->eyk2) + pid->eyk1 - pid->eyk2);

    pid->eyk2 = pid->eyk1;
    pid->eyk1 = eyk;
    pid->ek1 = err;
    pid->yk = pid->yk1 + pid->delta_k;

    return update_pid_output(pid);
}

int pid_clr(PidObj *pid)
{
    if(!pid) {
        return -1;
    }
    
    pid->yk = 0;
    pid->yk1 = 0;
    pid->delta_k = 0;
    pid->ek1 = 0;
    pid->edk1 = 0;
    pid->edk2 = 0;
    pid->eyk1 = 0;
    pid->eyk2 = 0;
    return 0;
}

int pid_calc_clr(PidObj *pid)
{
    if(!pid) {
        return -1;
    }

    pid->delta_k = 0;
    pid->ek1 = 0;
    pid->edk1 = 0;
    pid->edk2 = 0;
    pid->eyk1 = 0;
    pid->eyk2 = 0;
    return 0;
}
