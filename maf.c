/**
 * @ Author: luoqi
 * @ Create Time: 2024-07-22 15:37
 * @ Modified by: luoqi
 * @ Modified time: 2024-11-13 15:09
 * @ Description:
 */

#include "maf.h"

void maf_init(LpfMaf *filter, float *buf, int wsize)
{
    filter->wsize = wsize;
    filter->buf = buf;
    filter->sum = 0;
    filter->head = 0;
}

float maf_calc(LpfMaf *filter, float uk)
{
    filter->buf[filter->head] = uk;
    filter->head = (filter->head + 1) % filter->wsize;
    filter->sum += uk - filter->buf[filter->head];
    return (filter->sum / (float)filter->wsize);
}
