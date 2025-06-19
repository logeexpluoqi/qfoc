/**
 * @ Author: luoqi
 * @ Create Time: 2024-07-22 15:37
 * @ Modified by: luoqi
 * @ Modified time: 2025-02-17 15:00
 * @ Description:
 */

#include "maf.h"

int maf_init(LpfMaf *filter, qfp_t *buf, int wsize)
{
    if(!filter || !buf || wsize <= 0) {
        return -1;
    }
    filter->wsize = wsize;
    filter->buf = buf;
    filter->sum = 0;
    filter->head = 0;
    return 0;
}

qfp_t maf_calc(LpfMaf *filter, qfp_t z)
{
    if (!filter || !filter->buf || filter->wsize <= 0) {
        return 0;
    }
    filter->sum -= filter->buf[filter->head];
    
    filter->buf[filter->head] = z;
    filter->sum += z;
    
    filter->head = (filter->head + 1) % filter->wsize;
    
    return filter->sum / (qfp_t)filter->wsize;
}

int maf_reset(LpfMaf *filter)
{
    if (!filter || !filter->buf || filter->wsize <= 0) {
        return -1;
    }
    for (int i = 0; i < filter->wsize; i++) {
        filter->buf[i] = 0;
    }
    filter->sum = 0;
    filter->head = 0;
    return 0;
}
