/*
 * @Author: luoqi 
 * @Date: 2022-03-16 09:26:15 
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-11-29 09:53:46
 */

#ifndef _HPF_1ST_H
#define _HPF_1ST_H

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef qfp_t
typedef float qfp_t;
#endif

typedef struct _hpf_first_order
{
    qfp_t fc;       // cut-off frequency
    qfp_t y_k1;     // last output
    qfp_t alpha;    // filter coefficient
    qfp_t ts;       // samping period
    qfp_t u_k1;     // last input
}Hpf1stObj;

int hpf_1st_init(Hpf1stObj *filter, qfp_t fc, qfp_t fs);

qfp_t hpf_1st_calcu(Hpf1stObj *filter, qfp_t u_k);

int hpf_1st_fc_set(Hpf1stObj *filter, qfp_t fc);

int hpf_1st_reset(Hpf1stObj *filter);

#ifdef __cplusplus
 }
#endif

#endif