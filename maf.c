/**
 * @ Author: luoqi
 * @ Create Time: 2024-07-22 15:37
 * @ Modified by: luoqi
 * @ Modified time: 2025-02-11 00:23
 * @ Description:
 */

#include "maf.h"

int maf_init(LpfMaf *filter, qfp_t *cache, int wsize)
{
    if(!filter || !cache || wsize <= 0) {
        return -1;
    }
    filter->wsize = wsize;
    filter->cache = cache;
    filter->sum = 0;
    filter->head = 0;
    return 0;
}

qfp_t maf_calc(LpfMaf *filter, qfp_t z)
{
    if (!filter || !filter->cache || filter->wsize <= 0) {
        return 0;
    }
    filter->sum -= filter->cache[filter->head];
    
    filter->cache[filter->head] = z;
    filter->sum += z;
    
    filter->head = (filter->head + 1) % filter->wsize;
    
    return filter->sum / (qfp_t)filter->wsize;
}
