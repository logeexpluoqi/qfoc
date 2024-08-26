/**
 * @ Author: luoqi
 * @ Create Time: 2024-08-02 10:15
 * @ Modified by: luoqi
 * @ Modified time: 2024-08-16 23:23
 * @ Description:
 */

#include "qfoc.h"

static void *_memset(void *dest, int c, uint32_t n)
{
    if((dest == (void *)0) || (n == 0)) {
        return (void *)0;
    } else {
        char *pdest = (char *)dest;
        while(n--) {
            *pdest++ = c;
        }
        return dest;
    }
}

static inline float _rpm2deg(float rpm)
{
    return 6.0f * rpm;
}

static inline float _deg2rpm(float deg)
{
    return deg / 6.0f;
}

static inline float _fmodf(float x, float y)
{
    return (x - ((int)(x / y)) * y);
}

// static const float _QFOC_PI                = 3.141592653589793f;
// static const float _QFOC_SQRT3             = 1.732050807568877f;
// static const float _QFOC_SQRT3_BY_3        = 0.577350269189626f;
// static const float _QFOC_3_BY_2            = 1.5f;
static const float _QFOC_ONE_BY_SQRT3 = 0.577350269189626f;
static const float _QFOC_TWO_BY_SQRT3 = 1.154700538379252f;
static const float _QFOC_SQRT3_BY_2 = 0.866025403784439f;
static const float _QFOC_2_BY_3 = 0.666666666666667f;

#define _QFOC_ABS(x)     ((x) >= 0 ? (x) : (-x))
#define _QFOC_MAX(x, y)  ((x) >= (y) ? (x) : (y))
#define _QFOC_MIN(x, y)  ((x) <= (y) ? (x) : (y))

const float _QFOC_FSIN_MAGIC = 0.017453292519943295769236907684886f;

const float _sin_table[] = {
    0.0,                                    // sin(0)
    0.17364817766693034885171662676931 ,    // sin(10)
    0.34202014332566873304409961468226 ,    // sin(20)
    0.5 ,                                   // sin(30)
    0.64278760968653932632264340990726 ,    // sin(40)
    0.76604444311897803520239265055542 ,    // sin(50)
    0.86602540378443864676372317075294 ,    // sin(60)
    0.93969262078590838405410927732473 ,    // sin(70)
    0.98480775301220805936674302458952 ,    // sin(80)
    1.0                                     // sin(90)
};

const float _cos_table[] = {
    1.0 ,                                   // cos(0)
    0.99984769515639123915701155881391 ,    // cos(1)
    0.99939082701909573000624344004393 ,    // cos(2)
    0.99862953475457387378449205843944 ,    // cos(3)
    0.99756405025982424761316268064426 ,    // cos(4)
    0.99619469809174553229501040247389 ,    // cos(5)
    0.99452189536827333692269194498057 ,    // cos(6)
    0.99254615164132203498006158933058 ,    // cos(7)
    0.99026806874157031508377486734485 ,    // cos(8)
    0.98768834059513772619004024769344      // cos(9)
};

static float _fatan2(float y, float x)
{
    // a := min (|x|, |y|) / max (|x|, |y|)
    float abs_y = _QFOC_ABS(y);
    float abs_x = _QFOC_ABS(x);
    // inject FLT_MIN in denominator to avoid division by zero
    float a = _QFOC_MIN(abs_x, abs_y) / (_QFOC_MAX(abs_x, abs_y) + 1.17549e-38);
    // s := a * a
    float s = a * a;
    // r := ((-0.0464964749 * s + 0.15931422) * s - 0.327622764) * s * a + a
    float r = ((-0.0464964749f * s + 0.15931422f) * s - 0.327622764f) * s * a + a;
    // if |y| > |x| then r := 1.57079637 - r
    if(abs_y > abs_x) {
        r = 1.57079637f - r;
    }
    // if x < 0 then r := 3.14159274 - r
    if(x < 0.0f) {
        r = 3.14159274f - r;
    }
    // if y < 0 then r := -r
    if(y < 0.0f) {
        r = -r;
    }

    return r;
}

static float _fsind(float x)
{
    int s = 0;

    x = _fmodf(x, 360.0f);

    x = x < 0.0f ? x + 360.0f : x;

    if(x >= 180.0f) {
        s = -1;
        x = x - 180.0f;
    }

    x = (x > 90.0f) ? (180.0f - x) : x;

    int a = x * 0.1f;
    float b = x - 10 * a;

    float y = _sin_table[a] * _cos_table[(int)b] + b * _QFOC_FSIN_MAGIC * _sin_table[9 - a];

    y = (s < 0) ? -y : y;
    return y;
}

static inline float _fcosd(float x)
{
    return _fsind(x + 90.0f);
}

static inline float _fsqrt(float x)
{
    union {
        float f;
        int i;
    } conv;
    conv.f = x;
    conv.i = 0x5f3504f3 - (conv.i >> 1);
    x = conv.f;
    x = x * (1.5f - (0.5f * x * x * x));

    return 1.0f / x;
}

static int _svm(float alpha, float beta, float *ta, float *tb, float *tc)
{
    uint8_t sector = 0;
    if(beta >= 0.0f) {
        if(alpha >= 0.0f) {
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
        if(alpha >= 0.0f) {
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
    float t1 = 0.0f, t2 = 0.0f;
    switch(sector) {
    case 1:
        t1 = alpha - _QFOC_ONE_BY_SQRT3 * beta;
        t2 = _QFOC_TWO_BY_SQRT3 * beta;
        *ta = (1.0f - t1 - t2) * 0.5f;
        *tb = *ta + t1;
        *tc = *tb + t2;
        break;
    case 2:
        t1 = alpha + _QFOC_ONE_BY_SQRT3 * beta;
        t2 = -alpha + _QFOC_ONE_BY_SQRT3 * beta;
        *tb = (1.0f - t1 - t2) * 0.5f;
        *ta = *tb + t2;
        *tc = *ta + t1;
        break;
    case 3:
        t1 = _QFOC_TWO_BY_SQRT3 * beta;
        t2 = -alpha - _QFOC_ONE_BY_SQRT3 * beta;
        *tb = (1.0f - t1 - t2) * 0.5f;
        *tc = *tb + t1;
        *ta = *tc + t2;
        break;
    case 4:
        t1 = -alpha + _QFOC_ONE_BY_SQRT3 * beta;
        t2 = -_QFOC_TWO_BY_SQRT3 * beta;
        *tc = (1.0f - t1 - t2) * 0.5f;
        *tb = *tc + t2;
        *ta = *tb + t1;
        break;
    case 5:
        t1 = -alpha - _QFOC_ONE_BY_SQRT3 * beta;
        t2 = alpha - _QFOC_ONE_BY_SQRT3 * beta;
        *tc = (1.0f - t1 - t2) * 0.5f;
        *ta = *tc + t1;
        *tb = *ta + t2;
        break;
    case 6:
        t1 = -_QFOC_TWO_BY_SQRT3 * beta;
        t2 = alpha + _QFOC_ONE_BY_SQRT3 * beta;
        *ta = (1.0f - t1 - t2) * 0.5f;
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

static inline int _clarke_transform(float a, float b, float c, float *alpha, float *beta)
{
    *alpha = (a - 0.5f * (b + c)) * _QFOC_2_BY_3;
    *beta = (_QFOC_SQRT3_BY_2 * (b - c)) * _QFOC_2_BY_3;
    return 0;
}

static inline int _iclarke_transform(float alpha, float beta, float *a, float *b, float *c)
{
    *a = alpha;
    *b = -0.5 * alpha + _QFOC_SQRT3_BY_2 * beta;
    *c = -0.5 * alpha - _QFOC_SQRT3_BY_2 * beta;
    return 0;
}

static inline int _park_transform(float alpha, float beta, float edegree, float *q, float *d)
{
    *q = -alpha * _fsind(edegree) + beta * _fcosd(edegree);
    *d = alpha * _fcosd(edegree) + beta * _fsind(edegree);
    return 0;
}

static inline int _ipark_transform(float q, float d, float edegree, float *alpha, float *beta)
{
    *alpha = d * _fcosd(edegree) - q * _fsind(edegree);
    *beta = d * _fsind(edegree) + q * _fcosd(edegree);
    return 0;
}

static int _qsvm_calc(float vbus, float q, float d, float edegree, float *ta, float *tb, float *tc)
{
    float alpha, beta;
    q = (q > vbus) ? vbus : ((q < -vbus) ? -vbus : q);
    d = (d > vbus) ? vbus : ((d < -vbus) ? -vbus : d);
    float _q = (q / vbus) * _QFOC_SQRT3_BY_2;
    float _d = (d / vbus) * _QFOC_SQRT3_BY_2;
    _ipark_transform(_q, _d, edegree, &alpha, &beta);
    return _svm(alpha, beta, ta, tb, tc);
}



int qfoc_init(QFoc *foc, PmsmMotor *motor, uint16_t pwm_max, float vbus_max, float deadzone, float imax, float vmax, float pmax, float pmin)
{
    int ret = 0;
    _memset(foc, 0, sizeof(QFoc));
    foc->motor = motor;
    if(foc->motor->gear_ratio <= 0.0f) {
        foc->motor->gear_ratio = 1.0f;
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
    if(foc->motor->rated_current <= 0.0f) {
        foc->motor->rated_current = 0.5f;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_MOTOR_PARAM;;
        ret = -1;
    }
    if(foc->motor->rated_voltage <= 0.0f) {
        foc->motor->rated_voltage = 0.5f;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_MOTOR_PARAM;;
        ret = -1;
    }
    if(foc->motor->kt <= 0.0f) {
        foc->motor->kt = 0.0f;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_MOTOR_PARAM;
        ret = -1;
    }
    foc->pwm_max = pwm_max;
    foc->vbus_max = vbus_max;
    foc->vmax = _rpm2deg(vmax);
    foc->pmax = pmax;
    foc->pmin = pmin;
    foc->deadzone = deadzone;
    if(imax <= 0.0f) {
        foc->imax = 0.5f;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_IMAX_NOT_SET;
        ret = -1;
    }
    foc->imax = imax;
    return ret;
}

int qfoc_enable(QFoc *foc, QFocEnable ena)
{
    if(ena == QFOC_ENABLE) {
        pid_clear(&foc->pid_p);
        pid_clear(&foc->pid_v);
        pid_clear(&foc->pid_iq);
        pid_clear(&foc->pid_id);
        foc->iqref = 0.0f;
        foc->idref = 0.0f;
        foc->vref = 0.0f;
        foc->pref = foc->p;
        foc->status = QFOC_STATUS_RUNNING;
        foc->err = QFOC_ERR_NONE;
    } else {
        foc->status = QFOC_STATUS_IDLE;
    }

    return 0;
}

int qfoc_ppid_init(QFoc *foc, float kp, float ki, float kd)
{
    return pid_init(&foc->pid_p, kp, ki, kd, foc->vmax);
}

int qfoc_vpid_init(QFoc *foc, float kp, float ki, float kd)
{
    return pid_init(&foc->pid_v, kp, ki, kd, foc->imax);
}

int qfoc_iqpid_init(QFoc *foc, float kp, float ki, float kd)
{
    return pid_init(&foc->pid_iq, kp, ki, kd, foc->imax);
}

int qfoc_idpid_init(QFoc *foc, float kp, float ki, float kd)
{
    return pid_init(&foc->pid_id, kp, ki, kd, foc->imax);
}

int qfoc_ppid_set(QFoc *foc, float kp, float ki, float kd)
{
    return pid_param_set(&foc->pid_p, kp, ki, kd);
}

int qfoc_vpid_set(QFoc *foc, float kp, float ki, float kd)
{
    pid_calc_clear(&foc->pid_v);
    return pid_param_set(&foc->pid_v, kp, ki, kd);
}

int qfoc_iqpid_set(QFoc *foc, float kp, float ki, float kd)
{
    pid_calc_clear(&foc->pid_iq);
    return pid_param_set(&foc->pid_iq, kp, ki, kd);
}

int qfoc_idpid_set(QFoc *foc, float kp, float ki, float kd)
{
    pid_calc_clear(&foc->pid_id);
    return pid_param_set(&foc->pid_id, kp, ki, kd);
}

int qfoc_vbus_update(QFoc *foc, float vbus)
{
    if(vbus > foc->vbus_max) {
        foc->vbus = foc->vbus_max;
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_OVBUS;
    }
    foc->vbus = vbus;
    return 0;
}

int qfoc_i_update(QFoc *foc, float ia, float ib, float ic)
{
    float alpha, beta;
    foc->ia = ia;
    foc->ib = ib;
    foc->ic = ic;
    _clarke_transform(ia, ib, ic, &alpha, &beta);
    _park_transform(alpha, beta, foc->edegree, &foc->iq, &foc->id);
    if((foc->iq > foc->imax) || (foc->iq < -foc->imax) || (foc->id > foc->imax) || (foc->id < -foc->imax)) {
        foc->iq = (foc->iq > foc->imax) ? foc->imax : ((foc->iq < -foc->imax) ? -foc->imax : foc->iq);
        foc->id = (foc->id > foc->imax) ? foc->imax : ((foc->id < -foc->imax) ? -foc->imax : foc->id);
        foc->status = QFOC_STATUS_ERROR;
        foc->err = QFOC_ERR_OIMAX;
        return -1;
    }
    return 0;
}

int qfoc_v_update(QFoc *foc, float v)
{
    if(foc->vmax != 0.0f) {
        if((v > foc->vmax) || (v < -foc->vmax)) {
            foc->v = (v > foc->vmax) ? foc->vmax : ((v < -foc->vmax) ? -foc->vmax : v);
            foc->status = QFOC_STATUS_ERROR;
            foc->err = QFOC_ERR_OVMAX;
            return -1;
        }
    }
    foc->v = v;
    return 0;
}

int qfoc_p_update(QFoc *foc, float p)
{
    if((foc->pmax != 0.0f) && (foc->pmin != 0.0f)) {
        if((p > foc->pmax) || (p < foc->pmin)) {
            foc->p = (p > foc->pmax) ? foc->pmax : ((p < foc->pmin) ? foc->pmin : p);
            foc->status = QFOC_STATUS_ERROR;
            foc->err = QFOC_ERR_OPMAX;
            return -1;
        }
    }
    foc->edegree = _fmodf(p * foc->motor->poles_pairs, 360.0f);
    if(foc->edegree < 0.0f) {
        foc->edegree += 360.0f;
    }
    foc->p = p;
    return 0;
}

/* FOC controller reference input set */
int qfoc_iref_set(QFoc *foc, float iqref, float idref)
{
    foc->iqref = (iqref > foc->imax) ? foc->imax : ((iqref < -foc->imax) ? -foc->imax : iqref);
    foc->idref = (idref > foc->imax) ? foc->imax : ((idref < -foc->imax) ? -foc->imax : idref);
    return 0;
}

int qfoc_vref_set(QFoc *foc, float vref)
{
    if(foc->vmax != 0.0f) {
        foc->vref = (vref > foc->vmax) ? foc->vmax : ((vref < -foc->vmax) ? -foc->vmax : vref);
    } else {
        foc->vref = vref;
    }
    return 0;
}

int qfoc_pref_set(QFoc *foc, float pref)
{
    if((foc->pmax == 0.0f) && (foc->pmin == 0.0f)) {
        foc->pref = pref;
    } else {
        foc->pref = (pref > foc->pmax) ? foc->pmax : ((pref < foc->pmin) ? foc->pmin : pref);
    }
    return 0;
}

int qfoc_force_calc(float vbus, float q, float d, float edegree, uint16_t pwm_max, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc)
{
    float ta, tb, tc;

    edegree = _fmodf(edegree, 360.0f);
    edegree = (edegree < 0.0f) ? edegree + 360.0f : edegree;

    int sector = _qsvm_calc(vbus, q, d, edegree, &ta, &tb, &tc);
    *pwma = (uint16_t)(ta * pwm_max);
    *pwmb = (uint16_t)(tb * pwm_max);
    *pwmc = (uint16_t)(tc * pwm_max);
    return sector;
}

int qfoc_oloop_calc(QFoc *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc)
{
    float ta, tb, tc;
    float iq = foc->iqref;
    float id = foc->idref;

    if(foc->status != QFOC_STATUS_RUNNING) {
        return -1;
    }

    if(foc->deadzone != 0.0f) {
        iq = ((iq < 0.0f) && (iq > -foc->deadzone)) ? -foc->deadzone : ((iq > 0.0f) && (iq < foc->deadzone)) ? foc->deadzone : iq;
    }

    foc->sector = _qsvm_calc(foc->vbus, iq, id, foc->edegree, &ta, &tb, &tc);
    foc->pwma = (uint16_t)(ta * foc->pwm_max);
    foc->pwmb = (uint16_t)(tb * foc->pwm_max);
    foc->pwmc = (uint16_t)(tc * foc->pwm_max);

    *pwma = foc->pwma;
    *pwmb = foc->pwmb;
    *pwmc = foc->pwmc;
    return 0;
}

/* FOC current loop control */
int qfoc_iloop_calc(QFoc *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc)
{
    float iq, id;
    float ta, tb, tc;

    if(foc->status != QFOC_STATUS_RUNNING) {
        return -1;
    }

    iq = pid_calc(&foc->pid_iq, foc->iqref - foc->iq);
    id = pid_calc(&foc->pid_id, foc->idref - foc->id);
    if(foc->deadzone != 0.0f) {
        iq = ((iq < 0.0f) && (iq > -foc->deadzone)) ? -foc->deadzone : ((iq > 0.0f) && (iq < foc->deadzone)) ? foc->deadzone : iq;
    }

    foc->sector = _qsvm_calc(foc->vbus, iq, id, foc->edegree, &ta, &tb, &tc);
    foc->pwma = (uint16_t)(ta * foc->pwm_max);
    foc->pwmb = (uint16_t)(tb * foc->pwm_max);
    foc->pwmc = (uint16_t)(tc * foc->pwm_max);

    *pwma = foc->pwma;
    *pwmb = foc->pwmb;
    *pwmc = foc->pwmc;
    return 0;
}

/* FOC velocity/position current double loop pid control */
int qfoc_vloop_update(QFoc *foc, float di_limit)
{
    float iref;
    iref = pid_calc(&foc->pid_v, foc->vref - foc->v);
    if(di_limit != 0) {
        iref = iref > di_limit ? di_limit : iref;
        iref = iref < -di_limit ? -di_limit : iref;
    }
    foc->iqref = iref;
    foc->idref = 0;
    return 0;
}

int qfoc_ploop_update(QFoc *foc, float di_limit)
{
    float iref;
    iref = pid_calc(&foc->pid_p, foc->pref - foc->p);
    if(di_limit != 0) {
        iref = iref > di_limit ? di_limit : iref;
        iref = iref < -di_limit ? -di_limit : iref;
    }
    foc->iqref = iref;
    foc->idref = 0;
    return 0;
}

int qfoc_vploop_update(QFoc *foc, float dv_limit)
{
    float vref;
    vref = pid_calc(&foc->pid_p, foc->pref - foc->p);
    if(dv_limit != 0) {
        vref = vref > dv_limit ? dv_limit : vref;
        vref = vref < -dv_limit ? -dv_limit : vref;
    }
    foc->vref = vref;
    return 0;
}

int qfoc_phase_calib(QFoc *foc, float idmax, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc)
{
    int ret;
    float ta, tb, tc;
    foc->sector = 0;
    ret = _qsvm_calc(foc->vbus, 0.0f, idmax, 0, &ta, &tb, &tc);
    *pwma = (uint16_t)(ta * foc->pwm_max);
    *pwmb = (uint16_t)(tb * foc->pwm_max);
    *pwmc = (uint16_t)(tc * foc->pwm_max);
    return ret;
}
