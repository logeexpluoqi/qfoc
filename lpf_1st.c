/*
 * @Author: luoqi 
 * @Date: 2022-03-16 09:27:04 
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-11-29 09:54:57
 */

#include "lpf_1st.h"

#define LPF_PI  3.1415926

int lpf_1st_init(Lpf1stObj *filter, float fc, float ts)
{
    filter->alpha = (2 * LPF_PI * ts * fc) / (1 + 2 * LPF_PI * ts * fc);;
    filter->fc = fc;
    filter->ts = ts;
    filter->y_k1 = 0;
    return 0;
}

int lpf_1st_fc_set(Lpf1stObj *filter, float fc)
{
    filter->alpha = (2 * LPF_PI * filter->ts * fc) / (1 + 2 * LPF_PI * filter->ts * fc);
    filter->fc = fc;
    return 0;
}

float lpf_1st_calc(Lpf1stObj *filter, float u_k)
{
    float y_k = filter->alpha * u_k + (1 - filter->alpha) * filter->y_k1;
    filter->y_k1 = y_k;

    return y_k;
}

float lpf_1st_kcalc(Lpf1stObj *filter, float u_k, float ts)
{
    float alpha = (2 * LPF_PI * ts * filter->fc) / (1 + 2 * LPF_PI * ts * filter->fc);
    float y_k = alpha * u_k + (1 - alpha) * filter->y_k1;
    filter->y_k1 = y_k;
    return y_k;
}
