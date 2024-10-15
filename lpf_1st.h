/*
 * @Author: luoqi 
 * @Date: 2022-03-16 09:26:15 
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-11-29 09:51:22
 */

#ifndef _LPF_1ST_H
#define _LPF_1ST_H

#ifdef __cplusplus
 extern "C" {
#endif

typedef struct _lpf_first_order
{
    float fc;    // cut-off frequency
    float y_k1;  // last output
    float alpha; // filter coefficient
    float ts;     // samping period
}Lpf1stObj;

int lpf_1st_init(Lpf1stObj *filter, float fc, float ts);

int lpf_1st_fc_set(Lpf1stObj *filter, float fc);

/* ts is  time-constant */
float lpf_1st_calc(Lpf1stObj *filter, float u_k);

/* ts is time-varying */
float lpf_1st_kcalc(Lpf1stObj *filter, float u_k, float ts);

#ifdef __cplusplus
 }
#endif

#endif
