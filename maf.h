/**
 * @ Author: luoqi
 * @ Create Time: 2024-07-22 15:37
 * @ Modified by: luoqi
 * @ Modified time: 2025-02-17 15:00
 * @ Description:
 */

#ifndef _MAF_H
#define _MAF_H

#ifdef __cplusplus
 extern "C" {
#endif

#ifndef qfp_t
typedef float qfp_t;
#endif

typedef struct _lpf_sa
{
    int wsize;
    int head;
    qfp_t sum;
    qfp_t *buf;
} LpfMaf;

int maf_init(LpfMaf *filter, qfp_t *buf, int wsize);

qfp_t maf_calc(LpfMaf *filter, qfp_t z);

#ifdef __cplusplus
 }
#endif

#endif
