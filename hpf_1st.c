/*
 * @Author: luoqi
 * @Date: 2022-11-29 09:44:24
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-11-29 09:53:35
 */

#include "hpf_1st.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int hpf_1st_init(Hpf1stObj *filter, qfp_t fc, qfp_t fs)
{
    if(!filter || fs <= 0) {
        return -1;
    }
    qfp_t ts = 1 / fs;
    filter->alpha = 1 / (1 + 2 * M_PI * fc * ts);
    filter->ts = ts;
    filter->fc = fc;
    filter->u_k1 = 0;
    filter->y_k1 = 0;
    return 0;
}

qfp_t hpf_1st_calcu(Hpf1stObj *filter, qfp_t u_k)
{
    if(!filter) {
        return 0;
    }
    qfp_t y_k = filter->alpha * (u_k - filter->u_k1 + filter->y_k1);
    filter->y_k1 = y_k;
    filter->u_k1 = u_k;
    return y_k;
}

int hpf_1st_fc_set(Hpf1stObj *filter, qfp_t fc)
{
    if(!filter || fc <= 0) {
        return -1;
    }
    filter->fc = fc;
    filter->alpha = 1 / (1 + 2 * M_PI * fc * filter->ts);
    return 0;
}

int hpf_1st_reset(Hpf1stObj *filter)
{
    if(!filter) {
        return -1;
    }
    filter->y_k1 = 0;
    filter->u_k1 = 0;
    return 0;
}
