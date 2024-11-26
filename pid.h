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

typedef struct pid_structure
{
    float kp, ki, kd;
    float ek1;
    float edk1;  // last d(ek)
    float yk, yk1;
    float delta_k;

    /* used in incomplete differential PID */
    float edk2;
    /* used in incomplete differential 
        or integral separation threshold  */
    float alpha; // 0 < alpha < 1

    /* used in integral varible PID */
    float lth; // low threshold in integral varible PID
    float hth; // high threshold in integral varible PID

    /* used in differencail first PID */
    float eyk1; // last d(yk)
    float eyk2;

    float olimit;
} PidObj;

int pid_init(PidObj *pid, float kp, float ki, float kd, float olimit);

int pid_param_set(PidObj *pid, float kp, float ki, float kd);

/* output limit PID or basic PID */
float pid_calc(PidObj *pid, float err);

/* incomplete differential PID */
int pid_incplt_diff_init(PidObj *pid, float kp, float ki, float kd, float alpha, float olimit);

float pid_incplt_diff_calc(PidObj *ctrl, float err);

/* integral separation PID */
int pid_int_sep_init(PidObj *pid, float kp, float ki, float kd, float alpha, float olimit);

float pid_int_sep_calc(PidObj *pid, float err);

/* integral varible PID */
int pid_int_var_init(PidObj *pid, float kp, float ki, float kd, float lth, float hth, float olimit);

float pid_int_var_calc(PidObj *pid, float err);

/* differential first PID */
int pid_diff_first_init(PidObj *pid, float kp, float ki, float kd, float alpha, float olimit);

float pid_diff_first_calc(PidObj *pid, float err);

/* incomplete differential and integral varible PID */
int pid_incplt_diff_int_var_init(PidObj *pid, float kp, float ki, float kd, float alpha, float lth, float hth, float olimit);

float pid_incplt_diff_int_var_calc(PidObj *ctrl, float err);

/* differential first and integral varible PID */
int pid_diff_first_int_var_init(PidObj *pid, float kp, float ki, float kd, float alpha, float lth, float hth, float olimit);

float pid_diff_first_int_var_calc(PidObj *pid, float err);

int pid_clr(PidObj *pid);

int pid_calc_clr(PidObj *pid);

#ifdef __cplusplus
}
#endif

#endif

