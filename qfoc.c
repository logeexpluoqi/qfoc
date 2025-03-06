/**
 * @ Author: luoqi
 * @ Create Time: 2024-08-02 10:15
 * @ Modified by: luoqi
 * @ Modified time: 2025-03-06 15:48
 * @ Description:
 */

#include "qfoc.h"

static inline  qfp_t _clamp(qfp_t val, qfp_t min, qfp_t max)
{
    return (val > max) ? max : ((val < min) ? min : val);
}

static void *_memset(void *dest, int c, uint32_t n)
{
    if(!dest) {
        return QNULL;
    }

    if(n == 0) {
        return dest;
    }

    uint8_t *pdest = (uint8_t *)dest;
    uint8_t value = (uint8_t)c;

    if(n >= 4 && ((uintptr_t)pdest & 3) == 0) {
        uint32_t fill = value | (value << 8) | (value << 16) | (value << 24);
        uint32_t *pdest32 = (uint32_t *)pdest;

        while(n >= 4) {
            *pdest32++ = fill;
            n -= 4;
        }

        pdest = (uint8_t *)pdest32;
    }

    while(n--) {
        *pdest++ = value;
    }

    return dest;
}

static inline qfp_t _rpm2deg(qfp_t rpm)
{
    return 6 * rpm;
}

static inline qfp_t _deg2rpm(qfp_t deg)
{
    return deg / 6;
}

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

// static const qfp_t _QFOC_PI                = 3.141592653589793f;
// static const qfp_t _QFOC_SQRT3             = 1.732050807568877f;
// static const qfp_t _QFOC_SQRT3_BY_3        = 0.577350269189626f;
// static const qfp_t _QFOC_3_BY_2            = 1.5f;
static const qfp_t _QFOC_ONE_BY_SQRT3 = 0.577350269189626f;
static const qfp_t _QFOC_TWO_BY_SQRT3 = 1.154700538379252f;
static const qfp_t _QFOC_SQRT3_BY_2 = 0.866025403784439f;
static const qfp_t _QFOC_2_BY_3 = 0.666666666666667f;

static inline qfp_t _abs(qfp_t x)
{
    return (x >= 0) ? x : -x;
}

static inline qfp_t _max(qfp_t x, qfp_t y)
{
    return (x >= y) ? x : y;
}

static inline qfp_t _min(qfp_t x, qfp_t y)
{
    return (x <= y) ? x : y;
}

#define _DEG2RAD (0.017453292519943295769236907684886f)

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

static qfp_t _fatan2(qfp_t y, qfp_t x)
{
    // a := min (|x|, |y|) / max (|x|, |y|)
    qfp_t abs_y = _abs(y);
    qfp_t abs_x = _abs(x);
    // inject FLT_MIN in denominator to avoid division by zero
    qfp_t a = _min(abs_x, abs_y) / (_max(abs_x, abs_y) + 1.17549e-38);
    // s := a * a
    qfp_t s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    qfp_t r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if(abs_y > abs_x) {
        r = 1.57079637f - r;
    }
    // if x < 0 then r := 3.14159274 - r
    if(x < 0) {
        r = 3.14159274f - r;
    }
    // if y < 0 then r := -r
    if(y < 0) {
        r = -r;
    }

    return r;
}

static qfp_t _fsin(qfp_t deg)
{
    deg = _fmodf(deg, 360);

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

static inline qfp_t _fcos(qfp_t deg)
{
    return _fsin(deg + 90);
}

static inline qfp_t _fsqrt(qfp_t x)
{
    union {
        qfp_t f;
        int i;
    } conv;
    qfp_t x2;
    x2 = x * 0.5f;
    conv.f = x;
    conv.i = 0x5f3504f3 - (conv.i >> 1);
    x = conv.f;
    x = x * (1.5f - (x2 * x * x)); // 1st iteration
    x = x * (1.5f - (x2 * x * x)); // 2nd iteration

    return 1 / x;
}

static int _svm(qfp_t alpha, qfp_t beta, qfp_t *ta, qfp_t *tb, qfp_t *tc)
{
    if(!ta || !tb || !tc) {
        return -1;
    }
    uint8_t sector = 0;
    if(beta >= 0) {
        if(alpha >= 0) {
            if(alpha < beta * _QFOC_ONE_BY_SQRT3) {
                sector = 2;
            } else {
                sector = 1;
            }
        } else {
            if(alpha < -beta * _QFOC_ONE_BY_SQRT3) {
                sector = 3;
            } else {
                sector = 2;
            }
        }
    } else {
        if(alpha >= 0) {
            if(alpha < -beta * _QFOC_ONE_BY_SQRT3) {
                sector = 5;
            } else {
                sector = 6;
            }
        } else {
            if(alpha < beta * _QFOC_ONE_BY_SQRT3) {
                sector = 4;

            } else {
                sector = 5;
            }
        }
    }
    qfp_t t1 = 0, t2 = 0;
    switch(sector) {
    case 1:
        t1 = alpha - _QFOC_ONE_BY_SQRT3 * beta;
        t2 = _QFOC_TWO_BY_SQRT3 * beta;
        *ta = (1 - t1 - t2) * 0.5f;
        *tb = *ta + t1;
        *tc = *tb + t2;
        break;
    case 2:
        t1 = alpha + _QFOC_ONE_BY_SQRT3 * beta;
        t2 = -alpha + _QFOC_ONE_BY_SQRT3 * beta;
        *tb = (1 - t1 - t2) * 0.5f;
        *ta = *tb + t2;
        *tc = *ta + t1;
        break;
    case 3:
        t1 = _QFOC_TWO_BY_SQRT3 * beta;
        t2 = -alpha - _QFOC_ONE_BY_SQRT3 * beta;
        *tb = (1 - t1 - t2) * 0.5f;
        *tc = *tb + t1;
        *ta = *tc + t2;
        break;
    case 4:
        t1 = -alpha + _QFOC_ONE_BY_SQRT3 * beta;
        t2 = -_QFOC_TWO_BY_SQRT3 * beta;
        *tc = (1 - t1 - t2) * 0.5f;
        *tb = *tc + t2;
        *ta = *tb + t1;
        break;
    case 5:
        t1 = -alpha - _QFOC_ONE_BY_SQRT3 * beta;
        t2 = alpha - _QFOC_ONE_BY_SQRT3 * beta;
        *tc = (1 - t1 - t2) * 0.5f;
        *ta = *tc + t1;
        *tb = *ta + t2;
        break;
    case 6:
        t1 = -_QFOC_TWO_BY_SQRT3 * beta;
        t2 = alpha + _QFOC_ONE_BY_SQRT3 * beta;
        *ta = (1 - t1 - t2) * 0.5f;
        *tc = *ta + t2;
        *tb = *tc + t1;
        break;
    default:
        *ta = 0;
        *tb = 0;
        *tc = 0;
        return -1;
    }
    return sector;
}

static inline int _clarke_transform(qfp_t a, qfp_t b, qfp_t c, qfp_t *alpha, qfp_t *beta)
{
    if(!alpha || !beta) {
        return -1;
    }
    *alpha = (a - 0.5f * (b + c)) * _QFOC_2_BY_3;
    *beta = (_QFOC_SQRT3_BY_2 * (b - c)) * _QFOC_2_BY_3;
    return 0;
}

static inline int _iclarke_transform(qfp_t alpha, qfp_t beta, qfp_t *a, qfp_t *b, qfp_t *c)
{
    if(!a || !b || !c) {
        return -1;
    }
    *a = alpha;
    *b = -0.5 * alpha + _QFOC_SQRT3_BY_2 * beta;
    *c = -0.5 * alpha - _QFOC_SQRT3_BY_2 * beta;
    return 0;
}

static inline int _park_transform(qfp_t alpha, qfp_t beta, qfp_t edegree, qfp_t *q, qfp_t *d)
{
    if(!q || !d) {
        return -1;
    }
    *q = -alpha * _fsin(edegree) + beta * _fcos(edegree);
    *d = alpha * _fcos(edegree) + beta * _fsin(edegree);
    return 0;
}

static inline int _ipark_transform(qfp_t q, qfp_t d, qfp_t edegree, qfp_t *alpha, qfp_t *beta)
{
    if(!alpha || !beta) {
        return -1;
    }
    *alpha = d * _fcos(edegree) - q * _fsin(edegree);
    *beta = d * _fsin(edegree) + q * _fcos(edegree);
    return 0;
}

static int _qsvm_calc(qfp_t vbus, qfp_t vq, qfp_t vd, qfp_t edegree, qfp_t *ta, qfp_t *tb, qfp_t *tc)
{
    if(!ta || !tb || !tc) {
        return -1;
    }
    qfp_t alpha, beta;
    vq = (vq > vbus) ? vbus : ((vq < -vbus) ? -vbus : vq);
    vd = (vd > vbus) ? vbus : ((vd < -vbus) ? -vbus : vd);
    qfp_t _vq = (vq / vbus) * _QFOC_SQRT3_BY_2;
    qfp_t _vd = (vd / vbus) * _QFOC_SQRT3_BY_2;
    _ipark_transform(_vq, _vd, edegree, &alpha, &beta);
    return _svm(alpha, beta, ta, tb, tc);
}



int qfoc_init(QFocObj *foc, PmsmMotor *motor, uint16_t pwm_max, qfp_t vbus_max, qfp_t cilimit, uint32_t i2t_times, qfp_t imax, qfp_t vel_max, qfp_t pos_max, qfp_t pos_min)
{
    if(!foc || !motor) {
        return -1;
    }
    int ret = 0;
    _memset(foc, 0, sizeof(QFocObj));
    foc->motor = motor;
    if(foc->motor->gear_ratio <= 0) {
        foc->motor->gear_ratio = 1;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_MOTOR_PARAM;;
        ret = -1;
    }
    if(foc->motor->poles_pairs < 1) {
        foc->motor->poles_pairs = 1;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_MOTOR_PARAM;;
        ret = -1;
    }
    if(foc->motor->rated_current <= 0) {
        foc->motor->rated_current = 0;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_MOTOR_PARAM;;
        ret = -1;
    }
    if(foc->motor->rated_voltage <= 0) {
        foc->motor->rated_voltage = 0;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_MOTOR_PARAM;;
        ret = -1;
    }
    if(foc->motor->kt <= 0) {
        foc->motor->kt = 0;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_MOTOR_PARAM;
        ret = -1;
    }
    foc->cilimit = cilimit;
    foc->i2t_times = i2t_times;
    foc->pwm_max = pwm_max;
    foc->vbus_max = vbus_max;
    foc->vel_max = vel_max;
    foc->pos_max = pos_max;
    foc->pos_min = pos_min;
    if(imax <= 0) {
        foc->imax = 0;
        foc->iphase_max = 0;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_IMAX_NOT_SET;
        ret = -1;
    } else {
        foc->imax = imax;
        foc->iphase_max = imax;
    }
    return ret;
}

int qfoc_iloop_controller_set(QFocObj *foc, QFocError(*controller)(const QFocObj *foc, QFocOutput *output))
{
    if(!foc || !controller) {
        return -1;
    }

    foc->iloop_controller = controller;
    return 0;
}

int qfoc_ploop_controller_set(QFocObj *foc, QFocError(*controller)(const QFocObj *foc, QFocOutput *output))
{
    if(!foc || !controller) {
        return -1;
    }
    foc->ploop_controller = controller;
    return 0;
}

int qfoc_vloop_controller_set(QFocObj *foc, QFocError(*controller)(const QFocObj *foc, QFocOutput *output))
{
    if(!foc || !controller) {
        return -1;
    }
    foc->vloop_controller = controller;
    return 0;
}

int qfoc_enable(QFocObj *foc, QFocEnable ena)
{
    if(!foc) {
        return -1;
    }
    if(ena == QFOC_ENABLE) {
        foc->iqref = 0;
        foc->idref = 0;
        foc->vref = 0;
        foc->pref = foc->pos;
        foc->status = QFOC_STATUS_RUNNING;
        foc->err = QFOC_ERR_NONE;
    } else {
        foc->status = QFOC_STATUS_IDLE;
    }

    return 0;
}

int qfoc_vbus_set(QFocObj *foc, qfp_t vbus)
{
    if(!foc) {
        return -1;
    }
    if(vbus > foc->vbus_max) {
        foc->vbus = foc->vbus_max;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_OVBUS;
    } else {
        foc->vbus = vbus;
    }
    return 0;
}

int qfoc_i_set(QFocObj *foc, qfp_t ia, qfp_t ib, qfp_t ic)
{
    if(!foc) {
        return -1;
    }
    qfp_t alpha, beta;
    foc->ia = ia;
    foc->ib = ib;
    foc->ic = ic;

    if((_abs(ia) > foc->iphase_max) || (_abs(ib) > foc->iphase_max) || (_abs(ic) > foc->iphase_max)) {
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_OIMAX;
        return -1;
    }

    _clarke_transform(ia, ib, ic, &alpha, &beta);

    if(foc->i2t_cnt++ > foc->i2t_times) {
        foc->ci = foc->ipower / foc->i2t_times;
        foc->i2t_cnt = 0;
        foc->ipower = 0;
        if(foc->ci > foc->cilimit) {
            foc->status = QFOC_STATUS_ERROR;
            foc->err = QFOC_ERR_OPWR;
        }
    } else {
        foc->ipower += _fsqrt(foc->iq * foc->iq + foc->id * foc->id);
    }

    _park_transform(alpha, beta, foc->edegree, &foc->iq, &foc->id);
    if((foc->iq > foc->imax) || (foc->iq < -foc->imax) || (foc->id > foc->imax) || (foc->id < -foc->imax)) {
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_OIMAX;
        return -1;
    }
    return 0;
}

int qfoc_v_set(QFocObj *foc, qfp_t vel)
{
    if(!foc) {
        return -1;
    }
    foc->vel = vel;
    if(foc->vel_max != QFOC_NO_LIMIT) {
        if((vel > foc->vel_max) || (vel < -foc->vel_max)) {
            foc->status = QFOC_STATUS_ERROR;
            foc->err = QFOC_ERR_OVMAX;
            return -1;
        }
    }
    return 0;
}

int qfoc_epos_set(QFocObj *foc, qfp_t epos)
{
    if(!foc || !foc->motor || (foc->motor->gear_ratio <= 0)) {
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_MOTOR_PARAM;
        return -1;
    }
    foc->epos = epos;
    foc->pos = epos / foc->motor->gear_ratio;

    if((foc->pos_max != QFOC_NO_LIMIT) || (foc->pos_min != QFOC_NO_LIMIT)) {
        if((foc->pos > foc->pos_max) || (foc->pos < foc->pos_min)) {
            if(foc->pos > foc->pos_max) {
                foc->err = QFOC_ERR_OPMAX;
            } else {
                foc->err = QFOC_ERR_OPMIN;
            }
            foc->status = QFOC_STATUS_ERROR;
            return -1;
        }
    }
    foc->edegree = _fmodf(epos * foc->motor->poles_pairs, 360);
    if(foc->edegree < 0) {
        foc->edegree += 360;
    }
    return 0;
}

/* FOC controller reference input set */
int qfoc_vqd_set(QFocObj *foc, qfp_t vq, qfp_t vd)
{
    if(!foc) {
        return -1;
    }
    foc->vq = _clamp(vq, -foc->vbus, foc->vbus);
    foc->vd = _clamp(vd, -foc->vbus, foc->vbus);
    return 0;
}

int qfoc_iref_set(QFocObj *foc, qfp_t iqref, qfp_t idref)
{
    if(!foc) {
        return -1;
    }
    foc->iqref = _clamp(iqref, -foc->imax, foc->imax);
    foc->idref = _clamp(idref, -foc->imax, foc->imax);
    return 0;
}

int qfoc_vref_set(QFocObj *foc, qfp_t vref)
{
    if(!foc) {
        return -1;
    }
    if(foc->vel_max != QFOC_NO_LIMIT) {
        foc->vref = _clamp(vref, -foc->vel_max, foc->vel_max);
    } else {
        foc->vref = vref;
    }
    return 0;
}

int qfoc_pref_set(QFocObj *foc, qfp_t pref)
{
    if(!foc) {
        return -1;
    }
    if((foc->pos_max == QFOC_NO_LIMIT) && (foc->pos_min == QFOC_NO_LIMIT)) {
        foc->pref = pref;
    } else {
        foc->pref = _clamp(pref, foc->pos_min, foc->pos_max);
    }
    return 0;
}

int qfoc_force_calc(qfp_t vbus, qfp_t vq, qfp_t vd, qfp_t edegree, uint16_t pwm_max, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc)
{
    qfp_t ta, tb, tc;

    edegree = _fmodf(edegree, 360);
    edegree = (edegree < 0) ? edegree + 360 : edegree;

    int sector = _qsvm_calc(vbus, vq, vd, edegree, &ta, &tb, &tc);
    *pwma = (uint16_t)(ta * pwm_max);
    *pwmb = (uint16_t)(tb * pwm_max);
    *pwmc = (uint16_t)(tc * pwm_max);
    return sector;
}

int qfoc_oloop_update(QFocObj *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc)
{
    if(!foc) {
        return -1;
    }
    qfp_t ta, tb, tc;

    if(foc->status != QFOC_STATUS_RUNNING) {
        *pwma = 0;
        *pwmb = 0;
        *pwmc = 0;
        return -1;
    }
    if(foc->err != QFOC_ERR_NONE) {
        foc->status = QFOC_STATUS_ERROR;
        *pwma = 0;
        *pwmb = 0;
        *pwmc = 0;
        return -1;
    }

    foc->sector = _qsvm_calc(foc->vbus, foc->vq, foc->vd, foc->edegree, &ta, &tb, &tc);
    foc->pwma = (uint16_t)(ta * foc->pwm_max);
    foc->pwmb = (uint16_t)(tb * foc->pwm_max);
    foc->pwmc = (uint16_t)(tc * foc->pwm_max);

    *pwma = foc->pwma;
    *pwmb = foc->pwmb;
    *pwmc = foc->pwmc;
    return 0;
}

/* FOC current loop control */
int qfoc_iloop_update(QFocObj *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc)
{
    if(!foc) {
        return -1;
    }
    qfp_t ta, tb, tc;

    if(foc->status != QFOC_STATUS_RUNNING) {
        *pwma = 0;
        *pwmb = 0;
        *pwmc = 0;
        return 0;
    }
    if(foc->err != QFOC_ERR_NONE) {
        foc->status = QFOC_STATUS_ERROR;
        *pwma = 0;
        *pwmb = 0;
        *pwmc = 0;
        return -1;
    }
    QFocOutput output = { 0 };
    QFocError err = foc->iloop_controller(foc, &output);

    if(err != QFOC_ERR_NONE) {
        foc->status = QFOC_STATUS_ERROR;
        foc->err = err;
        *pwma = 0;
        *pwmb = 0;
        *pwmc = 0;
        return -1;
    }

    foc->sector = _qsvm_calc(foc->vbus, output.vq, output.vd, foc->edegree, &ta, &tb, &tc);
    foc->pwma = (uint16_t)(ta * foc->pwm_max);
    foc->pwmb = (uint16_t)(tb * foc->pwm_max);
    foc->pwmc = (uint16_t)(tc * foc->pwm_max);

    *pwma = foc->pwma > foc->pwm_max ? foc->pwm_max : foc->pwma;
    *pwmb = foc->pwmb > foc->pwm_max ? foc->pwm_max : foc->pwmb;
    *pwmc = foc->pwmc > foc->pwm_max ? foc->pwm_max : foc->pwmc;
    return 0;
}

/* FOC velocity/position current double loop pid control */
int qfoc_vloop_update(QFocObj *foc)
{
    if(!foc) {
        return -1;
    }
    QFocOutput output = { 0 };
    QFocError err = foc->vloop_controller(foc, &output);
    if(err != QFOC_ERR_NONE) {
        foc->status = QFOC_STATUS_ERROR;
        foc->err = err;
        return -1;
    }

    if(output.to == QFOC_OUT_TO_ILOOP) {
        foc->iqref = output.iqref;
        foc->idref = output.idref;
    } else if(output.to == QFOC_OUT_TO_OLOOP) {
        foc->vq = output.vq;
        foc->vd = output.vd;
    } else {
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_LOOP_OUTPUT;
        return -1;
    }

    return 0;
}

int qfoc_ploop_update(QFocObj *foc)
{
    if(!foc) {
        return -1;
    }
    QFocOutput output = { 0 };
    QFocError err = foc->ploop_controller(foc, &output);

    if(err != QFOC_ERR_NONE) {
        foc->status = QFOC_STATUS_ERROR;
        foc->err = err;
        return -1;
    }

    if(output.to == QFOC_OUT_TO_VLOOP) {
        foc->vref = output.vref;
    } else if(output.to == QFOC_OUT_TO_ILOOP) {
        foc->iqref = output.iqref;
        foc->idref = output.idref;
    } else if(output.to == QFOC_OUT_TO_OLOOP) {
        foc->vq = output.vq;
        foc->vd = output.vd;
    } else {
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_LOOP_OUTPUT;
        return -1;
    }
    return 0;
}

int qfoc_calib_calc(QFocObj *foc, qfp_t vdmax, qfp_t pref, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc)
{
    if(!foc || !pwma || !pwmb || !pwmc) {
        return -1;
    }
    int ret;
    qfp_t ta, tb, tc;
    foc->sector = 0;
    qfp_t edegree = _fmodf(pref * foc->motor->poles_pairs, 360);
    ret = _qsvm_calc(foc->vbus, 0, vdmax, edegree, &ta, &tb, &tc);
    *pwma = (uint16_t)(ta * foc->pwm_max);
    *pwmb = (uint16_t)(tb * foc->pwm_max);
    *pwmc = (uint16_t)(tc * foc->pwm_max);
    return ret;
}
