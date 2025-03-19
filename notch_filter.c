/**
 * @ Author: luoqi
 * @ Create Time: 2025-03-19 12:26
 * @ Modified by: luoqi
 * @ Modified time: 2025-03-19 14:06
 * @ Description:
 */

#include "notch_filter.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const qfp_t _fast_sin_table[91] = {
    0.0,           0.017452406,   0.034899497,   0.052335956,   0.069756474,
    0.087155743,   0.104528463,   0.121869343,   0.139173101,   0.156434465,
    0.173648178,   0.190809,      0.207911,      0.224951,      0.241922,
    0.258819,      0.275637,      0.292372,      0.309017,      0.325568,
    0.342020,      0.358368,      0.374607,      0.390731,      0.406737,
    0.422618,      0.438371,      0.453990,      0.469472,      0.48481,
    0.5,           0.515038,      0.529919,      0.544639,      0.559193,
    0.573576,      0.587785,      0.601815,      0.615661,      0.62932,
    0.642788,      0.656059,      0.669131,      0.681998,      0.694658,
    0.707107,      0.71934,       0.731354,      0.743145,      0.75471,
    0.766044,      0.777146,      0.788011,      0.798636,      0.809017,
    0.819152,      0.829038,      0.838671,      0.848048,      0.857167,
    0.866025,      0.87462,       0.882948,      0.891007,      0.898794,
    0.906308,      0.913545,      0.920505,      0.927184,      0.93358,
    0.939693,      0.945519,      0.951057,      0.956305,      0.961262,
    0.965926,      0.970296,      0.97437,       0.978148,      0.981627,
    0.984808,      0.987688,      0.990268,      0.992546,      0.994522,
    0.996195,      0.997564,      0.99863,       0.999391,      0.999848,
    1.0
};

static inline qfp_t _fmodf(qfp_t x, qfp_t y)
{
    if(y == 0) {
        return NAN;
    }

    qfp_t abs_y = y < 0 ? -y : y;
    qfp_t sign_x = x < 0 ? -1 : 1;
    qfp_t abs_x = x < 0 ? -x : x;

    qfp_t div = abs_x / abs_y;
    qfp_t int_part = div >= 0 ? (qfp_t)((int)div) : -(qfp_t)((int)-div);

    qfp_t result = abs_x - (int_part * abs_y);

    return sign_x * result;
}

static qfp_t _fcos(qfp_t deg)
{
    deg = _fmodf(deg + 90, 360);

    int sign = 1;
    // Use the periodic symmetry of sin
    if(deg > 180) {
        deg -= 180;
        sign = -1;
    }
    if(deg > 90) {
        deg = 180 - deg;
    }

    // Calculate the lookup table index and interpolation factor
    int index = (int)deg;  // 1° resolution
    if(index >= 90) {
        return sign * _fast_sin_table[90];
    }
    qfp_t fraction = deg - index;

    qfp_t sin_val = _fast_sin_table[index] * (1 - fraction) + _fast_sin_table[index + 1] * fraction;
    return sign * sin_val;
}

int notch_filter_init(NotchFilter *filter, qfp_t f0, qfp_t fs, qfp_t bw)
{
    if(!filter || (f0 <= 0) || (fs <= 0) || (bw <= 0) || (bw >= fs / 2)) {
        return -1;
    }
    qfp_t r = 1 - 3 * bw / fs;
    qfp_t cos_w0 = _fcos(2 * M_PI * f0 / fs);
    filter->a1 = -2 * cos_w0 * r;
    filter->a2 = r * r;
    filter->b0 = 1;
    filter->b1 = -2 * cos_w0;
    filter->b2 = 1;

    filter->x1 = 0;
    filter->x2 = 0;
    filter->y1 = 0;
    filter->y2 = 0;

    return 0;
}

qfp_t notch_filter_calc(NotchFilter *filter, qfp_t x)
{
    if(!filter) {
        return 0;
    }
    qfp_t y = filter->b0 * x + filter->b1 * filter->x1 + filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2 * filter->y2;
    filter->x2 = filter->x1;
    filter->x1 = x;
    filter->y2 = filter->y1;
    filter->y1 = y;
    return y;
}
