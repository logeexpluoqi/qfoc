/*
 * @Author: luoqi 
 * @Date: 2022-11-29 09:44:24 
 * @Last Modified by: luoqi
 * @Last Modified time: 2022-11-29 09:53:35
 */

#include "hpf_1st.h"

#define HPF_PI 3.1415926

int hpf_1st_init(Hpf1stObj *filter, float fc, float ts)
{
    filter->alpha = 1 / (1 + 2 * HPF_PI * fc * ts + 1);
    filter->ts = ts;
    filter->fc = fc;
    filter->u_k1 = 0;
    filter->y_k1 = 0;
    return 0;
}

float hpf_1st_calcu(Hpf1stObj *filter, float u_k)
{
    float y_k = filter->alpha * (u_k - filter->u_k1 + filter->y_k1);
    filter->y_k1 = y_k;
    filter->u_k1 = u_k;
    return y_k;
}
