/**
 * @ Author: luoqi
 * @ Create Time: 2024-11-08 17:16
 * @ Modified by: luoqi
 * @ Modified time: 2025-03-14 09:05
 * @ Description:
 */

#include "qwave.h"

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

static inline qfp_t fmodf_(qfp_t x, qfp_t y)
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

static qfp_t fsin_(qfp_t deg)
{
    deg = fmodf_(deg, 360);

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

static inline qfp_t fcos_(qfp_t x)
{
    return fsin_(x + 90);
}

static inline qfp_t sin_gen_(QWaveGen *gen)
{
    qfp_t x = (gen->t * 360) * gen->frq;
    gen->output = fsin_(x) + gen->bias;
    return gen->output;
}

static inline qfp_t tri_gen_(QWaveGen *gen)
{
    qfp_t norm = gen->t * gen->frq;

    if(norm < 0.25) {
        gen->output = 4 * norm;
    } else if(norm < 0.75) {
        gen->output = 2 - 4 * norm;
    } else {
        gen->output = 4 * norm - 4;
    }
    gen->output += gen->bias;
    return gen->output;
}

static inline qfp_t saw__gen_(QWaveGen *gen)
{
    gen->output = gen->t * gen->frq;
    return gen->output;
}

static inline qfp_t antsaw_gen_(QWaveGen *gen)
{
    gen->output = -(gen->t * gen->frq);
    gen->output += gen->bias;
    return gen->output;
}

static inline qfp_t sqr_gen_(QWaveGen *gen)
{
    // Square wave: first half period: +1; second half: -1.
    if(gen->t < gen->half_period) {
        gen->output = 1;
    } else {
        gen->output = -1;
    }
    gen->output += gen->bias;
    return gen->output;
}

static inline qfp_t noise_gen_(QWaveGen *gen)
{
    uint32_t x = gen->prng_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    gen->prng_state = x;
    // Map x (0 ~ 0xffffffffu) to [-1, 1]
    gen->output = ((qfp_t)x / 0xffffffffu) * 2 - 1 + gen->bias;
    return gen->output;
}

int qwave_init(QWaveGen *gen, QWaveType type, qfp_t fs, qfp_t frq, qfp_t bias, uint32_t seed)
{
    if(!gen || fs <= 0 || frq <= 0) {
        return -1;
    }

    gen->type = type;
    gen->bias = bias;
    gen->fs = fs;
    gen->frq = frq;
    gen->period = 1.0 / frq;
    gen->half_period = gen->period * 0.5;
    gen->ts = 1.0 / fs;
    gen->output = 0;
    gen->amp = 0.0;
    gen->t = 0;
    gen->seed = seed;
    gen->prng_state = (seed == 0) ? 2463534242UL : seed;

    return 0;
}

int qwave_signal_set(QWaveGen *gen, QWaveType type)
{
    if(!gen) {
        return -1;
    }
    gen->type = type;
    gen->t = 0;
    gen->output = 0;
    return 0;
}

static inline qfp_t _qwave_out(QWaveGen *gen)
{
    if(!gen) {
        return 0;
    }
    switch(gen->type) {
    case QWAVE_SINE:
        return sin_gen_(gen);
    case QWAVE_TRIANGLE:
        return tri_gen_(gen);
    case QWAVE_SAWTOOTH:
        return saw__gen_(gen);
    case QWAVE_ANTSAWTOOTH:
        return antsaw_gen_(gen);
    case QWAVE_SQUARE:
        return sqr_gen_(gen);
    case QWAVE_NOISE:
        return noise_gen_(gen);
    default:
        return 0;
    }
}

qfp_t qwave_gen(QWaveGen *gen)
{
    if(!gen) {
        return 0;
    }

    qfp_t out = _qwave_out(gen) * gen->amp;

    gen->t += gen->ts;
    gen->t = fmodf_(gen->t, gen->period);
    return out;
}

int qwave_bias_set(QWaveGen *gen, qfp_t bias)
{
    if(!gen) {
        return -1;
    }
    gen->bias = bias;
    return 0;
}

int qwave_fs_set(QWaveGen *gen, qfp_t fs)
{
    if(!gen || fs <= 0) {
        return -1;
    }
    gen->fs = fs;
    gen->ts = 1.0 / fs;
    return 0;
}

int qwave_frq_set(QWaveGen *gen, qfp_t frq)
{
    if(!gen || frq <= 0) {
        return -1;
    }
    gen->frq = frq;
    gen->period = 1.0 / frq;
    gen->half_period = gen->period * 0.5;
    return 0;
}

int qwave_amp_set(QWaveGen *gen, qfp_t amp)
{
    if(!gen || amp <= 0) {
        return -1;
    }
    gen->amp = amp / 2;
    return 0;
}

int qwave_reset(QWaveGen *gen)
{
    if(!gen) {
        return -1;
    }
    gen->t = 0;
    gen->output = 0;
    return 0;
}
