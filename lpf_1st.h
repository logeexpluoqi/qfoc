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

#ifndef qfp_t
typedef float qfp_t;
#endif

typedef struct _lpf_first_order
{
    qfp_t fc;    // cut-off frequency
    qfp_t y_k1;  // last output
    qfp_t alpha; // filter coefficient
    qfp_t ts;     // samping period
}Lpf1stObj;

int lpf_1st_init(Lpf1stObj *filter, qfp_t fc, qfp_t fs);

int lpf_1st_fc_set(Lpf1stObj *filter, qfp_t fc);

/* ts is  time-constant */
qfp_t lpf_1st_calc(Lpf1stObj *filter, qfp_t uk);

/* ts is time-varying */
qfp_t lpf_1st_kcalc(Lpf1stObj *filter, qfp_t uk, qfp_t ts);

#ifdef __cplusplus
 }
#endif

#endif
