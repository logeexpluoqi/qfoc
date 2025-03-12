/*
 * @Author: luoqi 
 * @Date: 2021-04-27 19:19:44 
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-01-26 17:03:39
 */

#ifndef _PID_H
#define _PID_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef qfp_t
typedef float qfp_t;
#endif

// Structure definition for PID controller
typedef struct pid_structure
{
    qfp_t kp;       // Proportional gain coefficient
    qfp_t ki;       // Integral gain coefficient
    qfp_t kd;       // Derivative gain coefficient
    qfp_t kaw;      // Anti-windup gain coefficient
    qfp_t i_acc;    // Integral accumulator
    qfp_t e_k1;     // Error at the previous time step
    qfp_t de_k1;    // Derivative of error at the previous time step
    qfp_t y_k;      // Current output
    qfp_t y_k1;     // Output at the previous time step
    qfp_t delta_k;  // Output increment

    // Used in incomplete differential PID
    qfp_t de_k2;    // Derivative of error at the previous previous time step
    // Used in incomplete differential or integral separation threshold
    qfp_t alpha;    // 0 < alpha < 1

    // Used in integral variable PID
    qfp_t l_th;     // Low threshold for integral variable PID
    qfp_t h_th;     // High threshold for integral variable PID

    // Used in differential first PID
    qfp_t ey_k1;    // Derivative of output at the previous time step
    qfp_t ey_k2;    // Derivative of output at the previous previous time step

    qfp_t olimit;   // Output limit
} PidObj;

#define PID_NONE    (0)

// Initialize a basic PID controller
int pid_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t olimit);

// Set parameters for a PID controller
int pid_param_set(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd);

// Calculate output for a basic PID controller
qfp_t pid_calc(PidObj *pid, qfp_t err, qfp_t dt);

// Initialize an anti-windup PID controller with back-calculation method
qfp_t pid_aw_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t kaw, qfp_t olimit);

// Calculate output for an anti-windup PID controller
qfp_t pid_aw_calc(PidObj *pid, qfp_t err, qfp_t dt);

// Initialize an incomplete differential PID controller
int pid_incplt_diff_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit);

// Calculate output for an incomplete differential PID controller
qfp_t pid_incplt_diff_calc(PidObj *ctrl, qfp_t err, qfp_t dt);

// Initialize an integral separation PID controller
int pid_integ_sep_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit);

// Calculate output for an integral separation PID controller
qfp_t pid_integ_sep_calc(PidObj *pid, qfp_t err, qfp_t dt);

// Initialize an integral variable PID controller
int pid_integ_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t l_th, qfp_t h_th, qfp_t olimit);

// Calculate output for an integral variable PID controller
qfp_t pid_integ_var_calc(PidObj *pid, qfp_t err, qfp_t dt);

// Initialize a differential first PID controller
int pid_diff_first_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit);

// Calculate output for a differential first PID controller
qfp_t pid_diff_first_calc(PidObj *pid, qfp_t err, qfp_t dt);

// Initialize an incomplete differential and integral variable PID controller
int pid_incplt_diff_integ_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t lth, qfp_t hth, qfp_t olimit);

// Calculate output for an incomplete differential and integral variable PID controller
qfp_t pid_incplt_diff_integ_var_calc(PidObj *ctrl, qfp_t err, qfp_t dt);

// Initialize a differential first and integral variable PID controller
int pid_diff_first_integ_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t lth, qfp_t hth, qfp_t olimit);

// Calculate output for a differential first and integral variable PID controller
qfp_t pid_diff_first_integ_var_calc(PidObj *pid, qfp_t err, qfp_t dt);

// Clear the PID controller's state
int pid_clr(PidObj *pid);

// Clear the PID controller's state and calculate output
int pid_calc_clr(PidObj *pid);

#ifdef __cplusplus
}
#endif

#endif