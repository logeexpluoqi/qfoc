/**
 * Author: luoqi
 * Created Date: 2026-03-07 02:12:33
 * Last Modified: 2026-03-25 00:25:26
 * Modified By: luoqi at <**@****>
 * Copyright (c) 2026 <*****>
 * Description:
 */

#ifndef _PID_H_
#define _PID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

#ifndef fp_t
typedef float fp_t;
#endif

typedef enum {
    PID_DIFF_NORMAL,
    PID_DIFF_FIRST,
} PidDiffMd;

typedef struct {
    fp_t kp;  // proportional gain
    fp_t ki;  // integral gain
    fp_t kd;  // derivative gain
    fp_t kaw; // anti-windup gain, default: 1/ki

    fp_t coef_ki;   // integral term coefficient, ki * ts
    fp_t coef_kaw;  // anti-windup term coefficient, kaw * ts

    fp_t omax;  // maximum output
    fp_t omin;  // minimum output

    fp_t ts;            // sampling period
    fp_t fc;            // derivative low-pass filter cutoff frequency, unit: Hz
    fp_t coef_a;        // derivative low-pass filter coefficient a
    fp_t coef_b;        // derivative low-pass filter coefficient b
    fp_t r_diff_k1;     // previous difference input
    fp_t y_diff_k1;     // previous difference output
    fp_t y_unsat_k1;    // previous unsaturated output
    fp_t y_k1;          // previous output

    fp_t integ;         // integral term
    fp_t fdbk_k1;       // previous feedback
    PidDiffMd diff_md;  // differential mode
} Pid;

/** 
 * @brief initialize PID controller
 * @param pid PID controller
 * @param ts sampling period, unit: s
 * @param fc derivative low-pass filter cutoff frequency, unit: Hz
 * @param omax maximum output
 * @param omin minimum output
 * @param md differential mode
 * @return 0 if success, -1 if failed
 */
int pid_init(Pid *pid, fp_t ts, fp_t fc, fp_t omax, fp_t omin, PidDiffMd md);

/** 
 * @brief set PID gains
 * @param pid PID controller
 * @param kp proportional gain
 * @param ki integral gain, kaw will be set to ki by default
 * @param kd derivative gain
 * @return 0 if success, -1 if failed
 */
int pid_gain_set(Pid *pid, fp_t kp, fp_t ki, fp_t kd);

/** 
 * @brief set derivative low-pass filter cutoff frequency
 * @param pid PID controller
 * @param fc derivative low-pass filter cutoff frequency, unit: Hz
 * @return 0 if success, -1 if failed
 */
int pid_fc_set(Pid *pid, fp_t fc);

/** 
 * @brief set anti-windup gain, default: 1/ki
 * @param pid PID controller
 * @param kaw anti-windup gain
 * @return 0 if success, -1 if failed
 */
int pid_kaw_set(Pid *pid, fp_t kaw);

/** 
 * @brief calculate PID output
 * @param pid PID controller
 * @param ref reference input
 * @param fdbk feedback input
 * @return PID output
 */
fp_t pid_calc(Pid *pid, fp_t ref, fp_t fdbk);

/** 
 * @brief reset PID controller
 * @param pid PID controller
 * @return 0 if success, -1 if failed
 */
int pid_reset(Pid *pid);

#ifdef __cplusplus
}
#endif

#endif
