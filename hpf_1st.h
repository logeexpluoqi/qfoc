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

typedef struct _hpf_first_order
{
    float fc;       // cut-off frequency
    float y_k1;     // last output
    float alpha;    // filter coefficient
    float ts;       // samping period
    float u_k1;     // last input
}Hpf1stObj;

int hpf_1st_init(Hpf1stObj *filter, float fc, float ts);

float hpf_1st_calcu(Hpf1stObj *filter, float u_k);

#ifdef __cplusplus
 }
#endif

#endif
