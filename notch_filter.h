/**
 * @ Author: luoqi
 * @ Create Time: 2025-03-19 12:26
 * @ Modified by: luoqi
 * @ Modified time: 2025-03-19 21:12
 * @ Description:
 */

#ifndef _NOTCH_FILTER_H_
#define _NOTCH_FILTER_H_

#ifndef qfp_t
typedef float qfp_t;
#endif

#ifndef NAN
#define NAN (0.0f / 0.0f)
#endif

typedef struct {
    qfp_t a1;
    qfp_t a2;
    qfp_t b0;
    qfp_t b1;
    qfp_t b2;
    qfp_t x1;
    qfp_t x2;
    qfp_t y1;
    qfp_t y2;

    qfp_t fs;
    qfp_t bw;
    qfp_t f0;
} NotchFilter;

int notch_filter_init(NotchFilter *filter, qfp_t f0, qfp_t fs, qfp_t bw);

qfp_t notch_filter_calc(NotchFilter *filter, qfp_t x);

int notch_filter_reset(NotchFilter *filter);

int notch_filter_fs_set(NotchFilter *filter, qfp_t fs);

int notch_filter_bw_set(NotchFilter *filter, qfp_t bw);

int notch_filter_f0_set(NotchFilter *filter, qfp_t f0);

#endif
