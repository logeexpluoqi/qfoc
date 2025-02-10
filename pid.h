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

typedef struct pid_structure
{
    qfp_t kp, ki, kd;
    qfp_t ek1;
    qfp_t edk1;  // last d(ek)
    qfp_t yk, yk1;
    qfp_t delta_k;

    /* used in incomplete differential PID */
    qfp_t edk2;
    /* used in incomplete differential 
        or integral separation threshold  */
    qfp_t alpha; // 0 < alpha < 1

    /* used in integral varible PID */
    qfp_t lth; // low threshold in integral varible PID
    qfp_t hth; // high threshold in integral varible PID

    /* used in differencail first PID */
    qfp_t eyk1; // last d(yk)
    qfp_t eyk2;

    qfp_t olimit;
} PidObj;

#define PID_NONE    (0)

int pid_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t olimit);

int pid_param_set(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd);

/* output limit PID or basic PID */
qfp_t pid_calc(PidObj *pid, qfp_t err, qfp_t dt);

/* incomplete differential PID */
int pid_incplt_diff_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit);

qfp_t pid_incplt_diff_calc(PidObj *ctrl, qfp_t err, qfp_t dt);

/* integral separation PID */
int pid_int_sep_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit);

qfp_t pid_int_sep_calc(PidObj *pid, qfp_t err, qfp_t dt);

/* integral varible PID */
int pid_int_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t lth, qfp_t hth, qfp_t olimit);

qfp_t pid_int_var_calc(PidObj *pid, qfp_t err, qfp_t dt);

/* differential first PID */
int pid_diff_first_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t olimit);

qfp_t pid_diff_first_calc(PidObj *pid, qfp_t err, qfp_t dt);

/* incomplete differential and integral varible PID */
int pid_incplt_diff_int_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t lth, qfp_t hth, qfp_t olimit);

qfp_t pid_incplt_diff_int_var_calc(PidObj *ctrl, qfp_t err, qfp_t dt);

/* differential first and integral varible PID */
int pid_diff_first_int_var_init(PidObj *pid, qfp_t kp, qfp_t ki, qfp_t kd, qfp_t alpha, qfp_t lth, qfp_t hth, qfp_t olimit);

qfp_t pid_diff_first_int_var_calc(PidObj *pid, qfp_t err, qfp_t dt);

int pid_clr(PidObj *pid);

int pid_calc_clr(PidObj *pid);

#ifdef __cplusplus
}
#endif

#endif

