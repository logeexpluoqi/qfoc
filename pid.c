/**
 * Author: luoqi
 * Created Date: 2026-03-07 23:27:34
 * Last Modified: 2026-03-20 23:28:26
 * Modified By: luoqi at <**@****>
 * Copyright (c) 2026 <*****>
 * Description:
 */

#include "pid.h"

static const fp_t pi = 3.14159265358979323846;

static inline fp_t clamp_(fp_t x, fp_t min, fp_t max)
{
    return x < min ? min : (x > max ? max : x);
}

int pid_init(Pid *pid, fp_t ts, fp_t fc, fp_t omax, fp_t omin, PidDiffMd md)
{
    if(!pid || (ts < 0.0) || (fc < 0.0)) {
        return -1;
    }
    pid->ts = ts;
    pid->fc = fc;
    pid->omax = omax;
    pid->omin = omin;
    pid->diff_md = md;
    pid->kaw = -1;
    pid->kp = 0.0;
    pid->ki = 0.0;
    pid->kd = 0.0;
    pid->coef_ki = 0.0;
    pid->coef_kaw = 0.0;

    pid_reset(pid);

    return 0;
}

int pid_gain_set(Pid *pid, fp_t kp, fp_t ki, fp_t kd)
{
    if(!pid || (kp < 0.0) || (ki < 0.0) || (kd < 0.0)) {
        return -1;
    }
    pid_reset(pid);
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->kaw = ki;
    pid->coef_ki = ki * pid->ts;

    if(pid->kaw < 0) {
        pid->kaw = ki;
        pid->coef_kaw = ki * pid->ts;
    }

    fp_t wc = 2.0 * pi * pid->fc;
    pid->coef_a = (2 * pid->kd * wc) / (2 + wc * pid->ts);
    pid->coef_b = (wc * pid->ts - 2) / (2 + wc * pid->ts);
    return 0;
}

int pid_kaw_set(Pid *pid, fp_t kaw)
{
    if(!pid || (kaw < 0.0)) {
        return -1;
    }
    pid_reset(pid);
    pid->kaw = kaw;
    pid->coef_kaw = kaw * pid->ts;
    return 0;
}

int pid_fc_set(Pid *pid, fp_t fc)
{
    if(!pid || (fc <= 0.0)) {
        return -1;
    }
    pid_reset(pid);
    pid->fc = fc;
    fp_t wc = 2.0 * pi * fc;
    pid->coef_a = (2 * pid->kd * wc) / (2 + wc * pid->ts);
    pid->coef_b = (wc * pid->ts - 2) / (2 + wc * pid->ts);
    return 0;
}

fp_t pid_calc(Pid *pid, fp_t ref, fp_t fdbk)
{
    if(!pid) {
        return 0.0;
    }
    fp_t e = ref - fdbk;
    fp_t p = pid->kp * e;
    fp_t d = 0.0;

    if(pid->diff_md == PID_DIFF_NORMAL) {
        d = pid->coef_a * (e - pid->r_diff_k1) - pid->coef_b * pid->y_diff_k1;
        pid->r_diff_k1 = e;
        pid->y_diff_k1 = d;
    } else {
        fp_t ext_e = fdbk - pid->fdbk_k1;
        pid->fdbk_k1 = fdbk;
        d = pid->coef_a * (ext_e - pid->r_diff_k1) - pid->coef_b * pid->y_diff_k1;
        pid->r_diff_k1 = ext_e;
        pid->y_diff_k1 = d;
    }

    fp_t y_unsat = p + pid->integ + d;

    fp_t y = clamp_(y_unsat, pid->omin, pid->omax);

    pid->integ = pid->integ + pid->coef_ki * e - pid->coef_kaw * (y_unsat - y);

    return y;
}

int pid_reset(Pid *pid)
{
    if(!pid) {
        return -1;
    }
    pid->r_diff_k1 = 0.0;
    pid->y_diff_k1 = 0.0;
    pid->integ = 0.0;
    pid->fdbk_k1 = 0.0;
    return 0;
}
