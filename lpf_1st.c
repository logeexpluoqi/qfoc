/*
 * @Author: luoqi 
 * @Date: 2022-03-16 09:27:04 
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-11-29 09:54:57
 */

#include "lpf_1st.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int lpf_1st_init(Lpf1stObj *filter, qfp_t fc, qfp_t fs)
{
    if (!filter || fs <= 0) {
        return -1;
    }
    qfp_t ts = 1.0f / fs;
    filter->alpha = (2 * M_PI * ts * fc) / (1 + 2 * M_PI * ts * fc);
    filter->fc = fc;
    filter->ts = ts;
    filter->y_k1 = 0;
    return 0;
}

int lpf_1st_fc_set(Lpf1stObj *filter, qfp_t fc)
{
    if (!filter || filter->ts <= 0) {
        return -1;
    }
    filter->alpha = (2 * M_PI * filter->ts * fc) / (1 + 2 * M_PI * filter->ts * fc);
    filter->fc = fc;
    return 0;
}

qfp_t lpf_1st_calc(Lpf1stObj *filter, qfp_t uk)
{
    if(!filter) {
        return 0;
    }
    qfp_t y_k = filter->alpha * uk + (1 - filter->alpha) * filter->y_k1;
    filter->y_k1 = y_k;

    return y_k;
}

qfp_t lpf_1st_kcalc(Lpf1stObj *filter, qfp_t uk, qfp_t fs)
{
    if(!filter || fs <= 0) {
        return 0;
    }
    qfp_t ts = 1.0f / fs;
    qfp_t alpha = (2 * M_PI * ts * filter->fc) / (1 + 2 * M_PI * ts * filter->fc);
    qfp_t y_k = alpha * uk + (1 - alpha) * filter->y_k1;
    filter->y_k1 = y_k;
    return y_k;
}
