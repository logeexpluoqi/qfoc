/*
 * @Author: luoqi
 * @Date: 2021-04-27 19:20:38
 * @ Modified by: luoqi
 * @ Modified time: 2025-03-12 19:36
 */

#include "pid.h"

static inline qfp_t _abs(qfp_t x)
{
    return x > 0 ? x : -x;
}

static inline qfp_t update_pid_output(PidObj *pid)
{
    if(pid->olimit != PID_NONE) {
        if(pid->y_k > pid->olimit) {
            pid->y_k = pid->olimit;
        } else if(pid->y_k < -pid->olimit) {
            pid->y_k = -pid->olimit;
        }
    }
    pid->y_k1 = pid->y_k;
    return pid->y_k;
}

int pid_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    pid->delta_k = 0;
    pid->e_k1 = 0;
    pid->de_k1 = 0;
    pid->de_k2 = 0;
    pid->y_k1 = 0;
    pid->y_k = 0;
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
    qfp_t de_k = (dt != PID_NONE) ? (err - pid->e_k1) / dt : err - pid->e_k1;
    pid->delta_k = pid->kp * (err - pid->e_k1)
        + pid->ki * err
        + pid->kd * (de_k - pid->de_k1);

    pid->de_k1 = de_k;
    pid->e_k1 = err;
    pid->y_k = pid->y_k1 + pid->delta_k;

    return update_pid_output(pid);
}

qfp_t pid_aw_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t kaw, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    pid_init(pid, kp, ki, kd, olimit);
    pid->kaw = kaw;
    return 0;
}

qfp_t pid_aw_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    // Calculate the derivative term (prevent dt from being 0 or special value)
    qfp_t de_k = (dt != PID_NONE) ? (err - pid->e_k1) / dt : err - pid->e_k1;

    // Calculate the proportional and derivative terms
    qfp_t p_term = pid->kp * (err - pid->e_k1);             // Proportional term
    qfp_t d_term = pid->kd * (de_k - pid->de_k1);           // Derivative term

    // unsat is the unsaturated output, composed of the previous output, integral accumulator, and P, D terms
    qfp_t unsat = pid->y_k1 + p_term + pid->i_acc + d_term;

    // Get the saturated output based on the set saturation limits
    qfp_t sat = unsat;
    if(pid->olimit != PID_NONE) {
        if(unsat > pid->olimit) {
            sat = pid->olimit;
        } else if(unsat < -pid->olimit) {
            sat = -pid->olimit;
        }
    }

    // Correct the integrator using reverse calculation: correction term = kaw * (sat - unsat)
    if(dt != PID_NONE) {
        pid->i_acc += (pid->ki * err - pid->kaw * (sat - unsat)) * dt; // Integral accumulator update with anti-windup
    } else {
        pid->i_acc += pid->ki * err - pid->kaw * (sat - unsat); // Integral accumulator update with anti-windup
    }

    // Update control quantity and state
    pid->delta_k = p_term + pid->i_acc + d_term;                // Total control increment
    pid->y_k = pid->y_k1 + pid->delta_k;                        // Current output

    pid->e_k1 = err;                                            // Update previous error
    pid->de_k1 = de_k;                                          // Update previous derivative of error
    pid->y_k1 = pid->y_k;                                       // Update previous output

    return update_pid_output(pid);
}

int pid_integ_sep_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }
    pid_init(pid, kp, ki, kd, olimit);
    if(alpha < 0 || alpha > 1) {
        pid->alpha = 1;
        return -1;
    }
    pid->alpha = alpha;
    return 0;
}

qfp_t pid_integ_sep_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    int th = _abs(err) > pid->alpha ? 0 : 1;        // Threshold for integral separation
    qfp_t de_k = (dt != PID_NONE) ? (err - pid->e_k1) / dt : err - pid->e_k1; // Derivative of error

    // Calculate the control increment with integral separation
    pid->delta_k = pid->kp * (err - pid->e_k1)      // Proportional term
        + th * pid->ki * err                        // Integral term with separation
        + pid->kd * (de_k - pid->de_k1);            // Derivative term

    pid->de_k1 = de_k;                              // Update previous derivative of error
    pid->e_k1 = err;                                // Update previous error
    pid->y_k = pid->y_k1 + pid->delta_k;            // Current output

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

    qfp_t de_k = (dt != PID_NONE) ? (err - pid->e_k1) / dt : err - pid->e_k1; // Derivative of error

    // Calculate the control increment with incomplete differential
    pid->delta_k = pid->kp * (err - pid->e_k1)          // Proportional term
        + pid->ki * err                                 // Integral term
        + pid->kd * (pid->alpha * (de_k - 2 * pid->de_k1 + pid->de_k2) + pid->de_k1 - pid->de_k2); // Incomplete differential term

    pid->de_k2 = pid->de_k1;                            // Update previous previous derivative of error
    pid->de_k1 = de_k;                                  // Update previous derivative of error
    pid->e_k1 = err;                                    // Update previous error
    pid->y_k = pid->y_k1 + pid->delta_k;                // Current output

    return update_pid_output(pid);
}

/* integral varible PID */
int pid_integ_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t l_th, qfp_t h_th, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }

    pid_init(pid, kp, ki, kd, olimit);

    if((l_th < 0) || (h_th < 0) || (l_th > h_th)) {
        pid->l_th = 1;
        return -1;
    }

    pid->l_th = l_th;
    pid->h_th = h_th;
    return 0;
}

qfp_t pid_integ_var_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    qfp_t ratio = 0;
    qfp_t de_k = (dt != PID_NONE) ? (err - pid->e_k1) / dt : err - pid->e_k1; // Derivative of error

    // Calculate the ratio based on error magnitude for integral variable PID
    if(_abs(err) <= pid->l_th) {
        ratio = 1;
    } else if(_abs(err) > pid->h_th) {
        ratio = 0;
    } else {                                        // l_th < |e| < h_th
        ratio = (pid->h_th - _abs(err)) / (pid->h_th - pid->l_th);
    }

    // Calculate the control increment with integral variable
    pid->delta_k = pid->kp * (err - pid->e_k1)      // Proportional term
        + ratio * pid->ki * err                     // Integral term with variable scaling
        + pid->kd * (de_k - pid->de_k1);            // Derivative term

    pid->de_k1 = de_k;                              // Update previous derivative of error
    pid->e_k1 = err;                                // Update previous error
    pid->y_k = pid->y_k1 + pid->delta_k;            // Current output

    return update_pid_output(pid);
}

int pid_diff_first_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit)
{
    if(!pid) {
        return -1;
    }

    pid_init(pid, kp, ki, kd, olimit);
    if(alpha < 0 || alpha > 1) {
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
    // Save the previous output of the derivative lead filter
    qfp_t ey_k1 = pid->ey_k1;
    
    // Calculate the current control increment:
    // Note: Retain the characteristics of the integral term (ki*err) and proportional term,
    // the derivative lead part utilizes the filter history variables (ey_k1, pid->ey_k2)
    pid->delta_k = pid->kp * (err - pid->e_k1)
        + pid->ki * err
        + pid->kd * (pid->alpha * (ey_k1 - 2 * pid->ey_k1 + pid->ey_k2) + pid->ey_k1 - pid->ey_k2);
    
    // Update the history state of the derivative lead filter: shift current output
    pid->ey_k2 = pid->ey_k1;
    pid->ey_k1 = pid->delta_k;
    
    pid->e_k1 = err;
    pid->y_k = pid->y_k1 + pid->delta_k;
    
    return update_pid_output(pid);
}

/* incomplete differential and integral varible PID */
int pid_incplt_diff_integ_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t l_th, qfp_t h_th, qfp_t olimit)
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
    if((l_th < 0) || (h_th < 0) || (l_th > h_th)) {
        pid->l_th = 1;
        return -1;
    }
    pid->l_th = l_th;
    pid->h_th = h_th;
    return 0;
}

qfp_t pid_incplt_diff_integ_var_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    qfp_t de_k = (dt != PID_NONE) ? (err - pid->e_k1) / dt : err - pid->e_k1; // Derivative of error
    qfp_t ratio = 0;

    // Calculate the ratio based on error magnitude for integral variable PID
    if(_abs(err) <= pid->l_th) {
        ratio = 1;
    } else if(_abs(err) > pid->h_th) {
        ratio = 0;
    } else { // l_th < |e| < h_th
        ratio = (pid->h_th - _abs(err)) / (pid->h_th - pid->l_th);
    }

    // Calculate the control increment with incomplete differential and integral variable
    pid->delta_k = pid->kp * (err - pid->e_k1)  // Proportional term
        + ratio * pid->ki * err                 // Integral term with variable scaling
        + pid->kd * (pid->alpha * (de_k - 2 * pid->de_k1 + pid->de_k2) + pid->de_k1 - pid->de_k2); // Incomplete differential term

    pid->de_k2 = pid->de_k1;                    // Update previous previous derivative of error
    pid->de_k1 = de_k;                          // Update previous derivative of error
    pid->e_k1 = err;                            // Update previous error
    pid->y_k = pid->y_k1 + pid->delta_k;        // Current output

    return update_pid_output(pid);
}

/* differential first and integral varible PID */
int pid_diff_first_integ_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t l_th, qfp_t h_th, qfp_t olimit)
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
    if((l_th < 0) || (h_th < 0) || (l_th > h_th)) {
        pid->l_th = 1;
        return -1;
    }
    pid->l_th = l_th;
    pid->h_th = h_th;
    return 0;
}

qfp_t pid_diff_first_integ_var_calc(PidObj *pid, qfp_t err, qfp_t dt)
{
    if(!pid) {
        return -1;
    }

    qfp_t eyk = pid->delta_k;                   // Previous control increment
    qfp_t ratio = 0;

    // Calculate the ratio based on error magnitude for integral variable PID
    if(_abs(err) <= pid->l_th) {
        ratio = 1;
    } else if(_abs(err) > pid->h_th) {
        ratio = 0;
    } else {                                    // l_th < |e| < h_th
        ratio = (pid->h_th - _abs(err)) / (pid->h_th - pid->l_th);
    }

    // Calculate the control increment with differential first and integral variable
    pid->delta_k = pid->kp * (err - pid->e_k1)  // Proportional term
        + ratio * pid->ki * err                 // Integral term with variable scaling
        + pid->kd * (pid->alpha * (eyk - 2 * pid->ey_k1 + pid->ey_k2) + pid->ey_k1 - pid->ey_k2); // Differential first term

    pid->ey_k2 = pid->ey_k1;                    // Update previous previous derivative of output
    pid->ey_k1 = eyk;                           // Update previous derivative of output
    pid->e_k1 = err;                            // Update previous error
    pid->y_k = pid->y_k1 + pid->delta_k;        // Current output

    return update_pid_output(pid);
}

int pid_clr(PidObj *pid)
{
    if(!pid) {
        return -1;
    }

    pid->y_k = 0;
    pid->y_k1 = 0;
    pid->delta_k = 0;
    pid->i_acc = 0;
    pid->e_k1 = 0;
    pid->de_k1 = 0;
    pid->de_k2 = 0;
    pid->ey_k1 = 0;
    pid->ey_k2 = 0;
    return 0;
}

int pid_calc_clr(PidObj *pid)
{
    if(!pid) {
        return -1;
    }

    pid->delta_k = 0;
    pid->e_k1 = 0;
    pid->i_acc = 0;
    pid->de_k1 = 0;
    pid->de_k2 = 0;
    pid->ey_k1 = 0;
    pid->ey_k2 = 0;
    return 0;
}
