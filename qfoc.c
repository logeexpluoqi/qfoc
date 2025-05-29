/**
 * @ Author: luoqi
 * @ Create Time: 2024-07-23 22:44
 * @ Modified by: luoqi
 * @ Modified time: 2025-04-11 19:26
 * @ Description:
 */

#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <bsp.h>
#include "foc.h"
#include "focc.h"
#include "../qshell/qterm.h"
#include "../modules/qfoc.h"
#include "../modules/qfsm.h"
#include "../controller/lpf_1st.h"
#include "../dev/qdrive.h"

#define QLOG_TAG    "FOC"
#include "../log/qlog.h"

typedef enum {
    FOC_INFO_NONE = 0,
    FOC_INFO_IQD,
    FOC_INFO_IABC,
    FOC_INFO_V,
    FOC_INFO_P,
    FOC_INFO_EDEGREE,
} FocInfoShowType;

typedef struct {
    bool showing;
    FocInfoShowType foc1_type;
    FocInfoShowType foc2_type;
} FocInfoShow;

static FocInfoShow foc_info_show;

static Lpf1stObj lpf_vbus;

typedef struct {
    Lpf1stObj iq;
    Lpf1stObj id;
} LpfIqdObj;
static LpfIqdObj ilpf_foc1;
static LpfIqdObj ilpf_foc2;

typedef struct {
    uint32_t vloop_cnt;
    uint32_t ploop_cnt;
    qtime_t last;
    qfp_t p_k1;
    qtime_t t_k1;
    Lpf1stObj vlpf;
} VpCalcInfo;

VpCalcInfo foc1_vpinfo, foc2_vpinfo;

static QFocObj *foc1, *foc2, *focobj;
static PmsmMotor *motor;
static QDrive *drv;

static Focc *focc;

static TX_THREAD task_foc1_vloop;
static uint8_t task_foc1_vloop_stack[1024];

static TX_THREAD task_foc1_ploop;
static uint8_t task_foc1_ploop_stack[1024];

static TX_THREAD task_foc2_vloop;
static uint8_t task_foc2_vloop_stack[1024];

static TX_THREAD task_foc2_ploop;
static uint8_t task_foc2_ploop_stack[1024];

static QCliCmd cmd_foc1;
static int cmd_foc_hdl(int argc, char **argv);
static int cmd_foc1_hdl(int argc, char **argv)
{
    focobj = foc1;
    motor = &drv->motor1;
    return cmd_foc_hdl(argc, argv);
}

static QCliCmd cmd_foc2;
static int cmd_foc2_hdl(int argc, char **argv)
{
    focobj = foc2;
    motor = &drv->motor2;
    return cmd_foc_hdl(argc, argv);
}

static QFocError focc1_iqd_loop(const QFocObj *foc, QFocOutput *output)
{
    return focc_iqd_loop(&focc->focc1, foc, output);
}

static QFocError focc2_iqd_loop(const QFocObj *foc, QFocOutput *output)
{
    return focc_iqd_loop(&focc->focc2, foc, output);
}

static QFocError focc1_vloop(const QFocObj *foc, QFocOutput *output)
{
    return focc_vloop(&focc->focc1, foc, output);
}

static QFocError focc2_vloop(const QFocObj *foc, QFocOutput *output)
{
    return focc_vloop(&focc->focc2, foc, output);
}

static QFocError focc1_ploop(const QFocObj *foc, QFocOutput *output)
{
    return focc_ploop(&focc->focc1, foc, output);
}

static QFocError focc2_ploop(const QFocObj *foc, QFocOutput *output)
{
    return focc_ploop(&focc->focc2, foc, output);
}

static int adc_init()
{
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 993);

    HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 993);

    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
    __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 993);

    HAL_TIM_OC_Start(&htim5, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 993);

    return 0;
}

static int enc_abz_init()
{
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_1 | TIM_CHANNEL_2);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    return 0;
}

int enc_abz_read(FocDev dev)
{
    int enc = 0;
    if(dev == FOC_DEV_1) {
        enc = -(__HAL_TIM_GET_COUNTER(&htim3));
    } else if(dev == FOC_DEV_2) {
        enc = (__HAL_TIM_GET_COUNTER(&htim4));
    } else {
        return 0;
    }
    return enc;
}

int enc_clr(FocDev dev)
{
    if(dev == FOC_DEV_1) {
        __HAL_TIM_SET_COUNTER(&htim3, 0);
    } else if(dev == FOC_DEV_2) {
        __HAL_TIM_SET_COUNTER(&htim4, 0);
    } else {
        return -1;
    }
    return 0;
}

static int foc_info_dump(const uint8_t *data, uint32_t len)
{
    uint8_t buf[20] = { 0 };
    uint32_t txsz = len + 4;
    if(txsz > sizeof(buf)) {
        QLOG_ERR("foc info dump buffer overflow");
        return -1;
    }
    memcpy(buf, data, len);
    buf[len] = 0x00;
    buf[len + 1] = 0x00;
    buf[len + 2] = 0x80;
    buf[len + 3] = 0x7f;
    uint32_t sz = bsp_usb_send(buf, txsz);
    if(sz != txsz) {
        QLOG_ERR("foc info dump failed, %d", sz);
    }
    return sz;
}

#define CALIB_ITERATIONS   100
#define CALIB_DELAY        1
#define CALIB_DELAY_AFTER  800 

static void _calib_pwm_loop(QFocObj *foc, float *id, float delta_i, int iterations, bool increasing)
{
    uint16_t pwma, pwmb, pwmc;
    for(int i = 0; i < iterations; i++) {
        if(increasing) {
            *id += delta_i;
        } else {
            *id -= delta_i;
        }
        qfoc_calib_calc(foc, *id, 0.0f, &pwma, &pwmb, &pwmc);
        if(foc == foc1) {
            foc_pwm_set(FOC_DEV_1, pwma, pwmb, pwmc);
        } else {
            foc_pwm_set(FOC_DEV_2, pwma, pwmb, pwmc);
        }
        tx_thread_sleep(CALIB_DELAY);
    }
}

static void axis_calib(QFocObj *foc, float imax)
{
    if(foc != foc1 && foc != foc2) {
        return;
    }

    imax = fmaxf(3.0f * foc->motor->rated_current, imax);
    float delta_i = imax / (float)CALIB_ITERATIONS;
    float id = 0;

    qfoc_enable(foc, QFOC_DISABLE);
    if(foc == foc1) {
        foc_pwm_start(FOC_DEV_1);
    } else {
        foc_pwm_start(FOC_DEV_2);
    }
    _calib_pwm_loop(foc, &id, delta_i, CALIB_ITERATIONS, true);
    tx_thread_sleep(CALIB_DELAY_AFTER);
    if(foc == foc1) {
        enc_clr(FOC_DEV_1);
    } else {
        enc_clr(FOC_DEV_2);
    }
    _calib_pwm_loop(foc, &id, delta_i, CALIB_ITERATIONS, false);
    if(foc == foc1) {
        foc_pwm_stop(FOC_DEV_1);
        drv->foc1_axis_aligned = true;
    } else {
        foc_pwm_stop(FOC_DEV_2);
        drv->foc2_axis_aligned = true;
    }
}

static inline void iloop_update(QFocObj *foc, FoccObj *focc, qfp_t adc_a, qfp_t adc_b, LpfIqdObj *lpf)
{
    /**
     * Phase current calculate:
     * ADC gain is 50, sampling resistor is 0.002ohm, adc vref is 3.3/2.0 = 1.5V,
     * ADC resolution is 12bit, <0-4096>
     * Reference voltage is 3.3V
     * so:
     *  v_adc = (adc_val / 4096) * 3.3V - 1.65V
     *  v = v_adc / 50
     *  i = v / 0.002ohm
     *    = (((adc_val / 4096) * 3.3V - 1.65v) / 50) / 0.002ohm
     * Note:
     *  sampling piont must be in SVM sector 0 or sector 7
    */
    float ia = (float)adc_a * 0.00805664f - 16.5f;
    float ib = (float)adc_b * 0.00805664f - 16.5f;
    float ic = -ia - ib;
    focc->iamax = fabsf(ia) > focc->iamax ? fabsf(ia) : focc->iamax;
    focc->ibmax = fabsf(ib) > focc->ibmax ? fabsf(ib) : focc->ibmax;
    focc->icmax = fabsf(ic) > focc->icmax ? fabsf(ic) : focc->icmax;

    qfoc_iabc_update(foc, ia, ib, ic);
    foc->iq = lpf_1st_calc(&lpf->iq, foc->iq);
    foc->id = lpf_1st_calc(&lpf->id, foc->id);

    focc->iqmax = fabsf(foc->iq) > focc->iqmax ? fabsf(foc->iq) : focc->iqmax;
    focc->idmax = fabsf(foc->id) > focc->idmax ? fabsf(foc->id) : focc->idmax;
    focc->cimax = fabsf(foc->ci) > focc->cimax ? fabsf(foc->ci) : focc->cimax;
}

static inline void vp_update(QFocObj *foc, FoccObj *focc, VpCalcInfo *vpinfo, FocDev dev, qtime_t now)
{
    float ep = (float)enc_abz_read(dev) * 0.02197265625f;
    ep = fmodf(ep, 360.0f);
    ep = (ep < 0.0f) ? (360.0f + ep) : ep;
    qfoc_epos_update(foc, ep); // (ep / (4096 * 4)) * 360
    qfp_t dp = ep - vpinfo->p_k1;
    dp = (dp > 180) ? (dp - 360) : ((dp < -180) ? (dp + 360) : dp);
    focc->position += dp;
    qfp_t dt = (qfp_t)(now - vpinfo->t_k1) * 1e-6f;
    qfp_t v = 0; 
    if(dt > 0) {
        v =  lpf_1st_calc(&vpinfo->vlpf, dp / dt);
        qfoc_vel_update(foc, v);
    }
    vpinfo->p_k1 = ep;
    vpinfo->t_k1 = now;
}

static inline void iloop_calc(QFocObj *foc, FocDev dev, bool axis_aligned)
{
    uint16_t pwma = 0, pwmb = 0, pwmc = 0;
    if(axis_aligned) {
        int ret = qfoc_iloop_update(foc, &pwma, &pwmb, &pwmc);
        if(ret < 0) {
            qfoc_enable(foc, QFOC_DISABLE);
        }
        foc_pwm_set(dev, pwma, pwmb, pwmc);
    }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    qtime_t now = bsp_ktime();

    if(hadc->Instance == ADC1) {
        drv->adc1_inj_cnt++;
        uint16_t adc_val_1 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        uint16_t adc_val_2 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);
        uint16_t adc_val_3 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_3);

        drv->vbus = lpf_1st_calc(&lpf_vbus, (float)adc_val_3 * 0.0169189453f) - drv->vbus_bias;
        qfoc_vbus_update(&drv->foc1, drv->vbus);
        qfoc_vbus_update(&drv->foc2, drv->vbus);

        iloop_update(&drv->foc1, &focc->focc1, adc_val_1, adc_val_2, &ilpf_foc1);
        iloop_calc(&drv->foc1, FOC_DEV_1, drv->foc1_axis_aligned);

        vp_update(&drv->foc1, &focc->focc1, &foc1_vpinfo, FOC_DEV_1, now);

        if(foc1_vpinfo.vloop_cnt++ > (5 - 1)) {
            foc1_vpinfo.vloop_cnt = 0;
            if(focc->focc1.cmod == FOCC_MOD_V || focc->focc1.cmod == FOCC_MOD_P) {
                tx_event_flags_set(&drv->foc_cloop_flags, QDRV_FOC1_VLOOP_FLAG, TX_OR);
            }
        }
        if(foc1_vpinfo.ploop_cnt++ > (21 - 1)) {
            foc1_vpinfo.ploop_cnt = 0;
            if(focc->focc1.cmod == FOCC_MOD_P) {
                tx_event_flags_set(&drv->foc_cloop_flags, QDRV_FOC1_PLOOP_FLAG, TX_OR);
            }
        }

        focc->focc1.iloop_us = bsp_ktime() - foc1_vpinfo.last;
        foc1_vpinfo.last = bsp_ktime();
        focc->focc1.iloop_calc_us = bsp_ktime() - now;
        if(foc_info_show.foc1_type == FOC_INFO_IQD) {
            qfp_t buf[2] = { foc1->iq, foc1->id };
            foc_info_dump((uint8_t *)buf, sizeof(buf));
        } else if(foc_info_show.foc1_type == FOC_INFO_IABC) {
            qfp_t buf[3] = { foc1->ia, foc1->ib, foc1->ic };
            foc_info_dump((uint8_t *)buf, sizeof(buf));
        } else {
            return;
        }
    }
    if(hadc->Instance == ADC2) {
        drv->adc2_inj_cnt++;
        uint16_t adc_val_3 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_1);
        uint16_t adc_val_4 = HAL_ADCEx_InjectedGetValue(hadc, ADC_INJECTED_RANK_2);

        iloop_update(&drv->foc2, &focc->focc2, adc_val_3, adc_val_4, &ilpf_foc2);
        iloop_calc(&drv->foc2, FOC_DEV_2, drv->foc2_axis_aligned);

        vp_update(&drv->foc2, &focc->focc2, &foc2_vpinfo, FOC_DEV_2, now);

        if(foc2_vpinfo.vloop_cnt++ > (5 - 1)) {
            foc2_vpinfo.vloop_cnt = 0;
            if(focc->focc2.cmod == FOCC_MOD_V || focc->focc2.cmod == FOCC_MOD_P) {
                tx_event_flags_set(&drv->foc_cloop_flags, QDRV_FOC2_VLOOP_FLAG, TX_OR);
            }
        }
        if(foc2_vpinfo.ploop_cnt++ > (21 - 1)) {
            foc2_vpinfo.ploop_cnt = 0;
            if(focc->focc2.cmod == FOCC_MOD_P) {
                tx_event_flags_set(&drv->foc_cloop_flags, QDRV_FOC2_PLOOP_FLAG, TX_OR);
            }
        }

        focc->focc2.iloop_us = bsp_ktime() - foc2_vpinfo.last;
        foc2_vpinfo.last = bsp_ktime();
        focc->focc2.iloop_calc_us = bsp_ktime() - now;
        if(foc_info_show.foc2_type == FOC_INFO_IQD) {
            qfp_t buf[2] = { foc2->iq, foc2->id };
            foc_info_dump((uint8_t *)buf, sizeof(buf));
        } else if(foc_info_show.foc2_type == FOC_INFO_IABC) {
            qfp_t buf[3] = { foc2->ia, foc2->ib, foc2->ic };
            foc_info_dump((uint8_t *)buf, sizeof(buf));
        } else {
            return;
        }
    }
}

static void _task_foc1_vloop_hdl(uint32_t arg)
{
    UNUSED(arg);
    axis_calib(foc1, 2);
    uint32_t flags;
    qtime_t last = 0;
    while(1) {
        qtime_t now = bsp_ktime();
        if(last == 0) {
            last = now;
        }

        tx_event_flags_get(&drv->foc_cloop_flags, QDRV_FOC1_VLOOP_FLAG, TX_OR_CLEAR, &flags, TX_WAIT_FOREVER);
        qfoc_vloop_update(foc1);

        focc->focc1.vloop_us = now - last;
        if(foc_info_show.foc1_type == FOC_INFO_V) {
            qfp_t v = foc1->vel;
            foc_info_dump((uint8_t *)&v, sizeof(v));
        }
        last = now;
    }
}

static void _task_foc2_vloop_hdl(uint32_t arg)
{
    UNUSED(arg);
    axis_calib(foc2, 3);
    uint32_t flags;
    qtime_t last = 0;
    while(1) {
        qtime_t now = bsp_ktime();
        if(last == 0) {
            last = now;
        }

        tx_event_flags_get(&drv->foc_cloop_flags, QDRV_FOC2_VLOOP_FLAG, TX_OR_CLEAR, &flags, TX_WAIT_FOREVER);
        qfoc_vloop_update(foc2);

        focc->focc2.vloop_us = now - last;
        if(foc_info_show.foc2_type == FOC_INFO_V) {
            qfp_t v = foc2->vel;
            foc_info_dump((uint8_t *)&v, sizeof(v));
        }
        last = bsp_ktime();
    }
}

static void _task_foc1_ploop_hdl(uint32_t arg)
{
    UNUSED(arg);
    qtime_t last = 0;
    uint32_t flags;
    while(1) {
        qtime_t t = bsp_ktime();
        if(last == 0) {
            last = t;
        }
        tx_event_flags_get(&drv->foc_cloop_flags, QDRV_FOC1_PLOOP_FLAG, TX_OR_CLEAR, &flags, TX_WAIT_FOREVER);
        qfoc_ploop_update(foc1);

        focc->focc1.ploop_us = t - last;
        last = t;
        if(foc_info_show.foc1_type == FOC_INFO_P) {
            qfp_t p = foc1->pos;
            foc_info_dump((uint8_t *)&p, sizeof(p));
        } else if(foc_info_show.foc1_type == FOC_INFO_EDEGREE) {
            qfp_t edegree = foc1->edegree;
            foc_info_dump((uint8_t *)&edegree, sizeof(edegree));
        } else {
            continue;
        }
    }
}

static void _task_foc2_ploop_hdl(uint32_t arg)
{
    UNUSED(arg);
    qtime_t last = 0;
    uint32_t flags;
    while(1) {
        qtime_t t = bsp_ktime();
        if(last == 0) {
            last = t;
        }

        tx_event_flags_get(&drv->foc_cloop_flags, QDRV_FOC2_PLOOP_FLAG, TX_OR_CLEAR, &flags, TX_WAIT_FOREVER);
        qfoc_ploop_update(foc2);

        focc->focc2.ploop_us = t - last;
        last = t;
        if(foc_info_show.foc2_type == FOC_INFO_P) {
            qfp_t p = foc2->pos;
            foc_info_dump((uint8_t *)&p, sizeof(p));
        } else if(foc_info_show.foc2_type == FOC_INFO_EDEGREE) {
            qfp_t edegree = foc2->edegree;
            foc_info_dump((uint8_t *)&edegree, sizeof(edegree));
        } else {
            continue;
        }
    }
}

static inline void bemf_update(FoccObj *focc, uint16_t *adc_val)
{
    focc->bemf_a = (float)adc_val[0] * 0.0169189453f;
    focc->bemf_b = (float)adc_val[1] * 0.0169189453f;
    focc->bemf_c = (float)adc_val[2] * 0.0169189453f;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    /**
     * Vbus calculate:
     * 1. adc_val is a 12bit adc sampling value.
     * 2. v_adc = (adc_val / 4096) * 3.3V.
     * 3. vbus = v_adc * sapling_resistor_ratio + bais.
     * Note:
     *  sampling_reistor_ratio = 21.0f
     */
    qtime_t now = bsp_ktime();
    if(hadc->Instance == ADC1) {
        drv->adc1_conv_cnt++;
        bemf_update(&focc->focc1, drv->adc1_buf);
        focc->focc1.bemf_calc_us = bsp_ktime() - now;
    }
    if(hadc->Instance == ADC2) {
        drv->adc2_conv_cnt++;
        bemf_update(&focc->focc2, drv->adc2_buf);
        focc->focc2.bemf_calc_us = bsp_ktime() - now;
    }
}

int foc_init(void)
{
    int ret = 0;
    drv = qdrv_get();
    foc1 = foc_obj_get(FOC_DEV_1);
    foc2 = foc_obj_get(FOC_DEV_2);

    foc_info_show.showing = false;
    foc_info_show.foc1_type = FOC_INFO_NONE;
    foc_info_show.foc2_type = FOC_INFO_NONE;

    lpf_1st_init(&ilpf_foc1.iq, 800, 21000);
    lpf_1st_init(&ilpf_foc1.id, 800, 21000);
    lpf_1st_init(&ilpf_foc2.iq, 800, 21000);
    lpf_1st_init(&ilpf_foc2.id, 800, 21000);
    lpf_1st_init(&lpf_vbus, 210, 21000);

    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_ADCEx_InjectedStart_IT(&hadc2);

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)drv->adc1_buf, 3);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)drv->adc2_buf, 3);

    adc_init();
    enc_abz_init();

    if(qfoc_init(foc1, &drv->motor1, 980, 28.0f, 1.5f, 10000, 3.0f, 0, 0.0f, 0.0f) < 0) {
        QLOG_ERR("#! foc1 init failed! status: %d, err: %d", foc1->status, foc1->err);
        return ret;
    }

    focc_init();
    focc = focc_get();
    qfoc_iloopc_set(foc1, focc1_iqd_loop);
    qfoc_vloopc_set(foc1, focc1_vloop);
    qfoc_ploopc_set(foc1, focc1_ploop);
    lpf_1st_init(&foc1_vpinfo.vlpf, 200, 21000);

    if(qfoc_init(foc2, &drv->motor2, 980, 28.0f, 1.8f, 10000, 3.0f, 0, 0.0f, 0.0f) < 0) {
        QLOG_ERR("#! foc2 init failed! status: %d, err: %d", foc2->status, foc2->err);
        return ret;
    }
    qfoc_iloopc_set(foc2, focc2_iqd_loop);
    qfoc_vloopc_set(foc2, focc2_vloop);
    qfoc_ploopc_set(foc2, focc2_ploop);
    lpf_1st_init(&foc2_vpinfo.vlpf, 200, 21000);

    tx_thread_create(&task_foc1_vloop, "foc1_vloop", _task_foc1_vloop_hdl, 0, task_foc1_vloop_stack, sizeof(task_foc1_vloop_stack), 5, 5, TX_NO_TIME_SLICE, TX_AUTO_START);
    tx_thread_create(&task_foc1_ploop, "foc1_ploop", _task_foc1_ploop_hdl, 0, task_foc1_ploop_stack, sizeof(task_foc1_ploop_stack), 6, 6, TX_NO_TIME_SLICE, TX_AUTO_START);
    tx_thread_create(&task_foc2_vloop, "foc2_vloop", _task_foc2_vloop_hdl, 0, task_foc2_vloop_stack, sizeof(task_foc2_vloop_stack), 5, 5, TX_NO_TIME_SLICE, TX_AUTO_START);
    tx_thread_create(&task_foc2_ploop, "foc2_ploop", _task_foc2_ploop_hdl, 0, task_foc2_ploop_stack, sizeof(task_foc2_ploop_stack), 6, 6, TX_NO_TIME_SLICE, TX_AUTO_START);
    qterm_attach(&cmd_foc1, "foc1", cmd_foc1_hdl, "foc1 task commands");
    qterm_attach(&cmd_foc2, "foc2", cmd_foc2_hdl, "foc2 task commands");

    return ret;
}

static inline void args_ena_common(FoccObj *focc, FocDev dev)
{
    focc->cmod = FOCC_MOD_IDLE;
    focc->iamax = 0.0f;
    focc->ibmax = 0.0f;
    focc->icmax = 0.0f;
    focc->iqmax = 0.0f;
    focc->idmax = 0.0f;
    focc->cimax = 0.0f;
    foc_pwm_start(dev);
    focc_controller_clr(focc);
}

static int args_ena_hdl(int argc, char **argv)
{
    UNUSED(argc);
    if(ISARG(argv[1], "1")) {
        qfoc_enable(focobj, QFOC_ENABLE);
        if(focobj == foc1) {
            args_ena_common(&focc->focc1, FOC_DEV_1);
        } else {
            args_ena_common(&focc->focc2, FOC_DEV_2);
        }
    } else if(ISARG(argv[1], "0")) {
        qfoc_enable(focobj, QFOC_ENABLE);
        if(focobj == foc1) {
            foc_pwm_stop(FOC_DEV_1);
        } else {
            foc_pwm_stop(FOC_DEV_2);
        }
    } else {
        return -1;
    }
    return 0;
}

int cmod_set(FocDev dev, FoccMod cmod)
{
    if(dev == FOC_DEV_1) {
        focc->focc1.cmod = cmod;
        drv->foc1.iqref = 0.0f;
        drv->foc1.idref = 0.0f;
        drv->foc1.vref = 0.0f;
        drv->foc1.pref = drv->foc1.pos;
        focc->focc1.position = drv->foc1.pos;
    } else if(dev == FOC_DEV_2) {
        focc->focc2.cmod = cmod;
        drv->foc2.iqref = 0.0f;
        drv->foc2.idref = 0.0f;
        drv->foc2.vref = 0.0f;
        drv->foc2.pref = drv->foc2.pos;
        focc->focc2.position = drv->foc2.pos;
    }
    return 0;
}

static int args_cmod_hdl(int argc, char **argv)
{
    UNUSED(argc);

    FoccMod cmod = FOCC_MOD_IDLE;
    if(ISARG(argv[1], "idle")) {
        cmod = FOCC_MOD_IDLE;
    } else if(ISARG(argv[1], "i")) {
        cmod = FOCC_MOD_I;
    } else if(ISARG(argv[1], "v")) {
        cmod = FOCC_MOD_V;
    } else if(ISARG(argv[1], "p")) {
        cmod = FOCC_MOD_P;
    } else {
        QSH(" -mod <idle/i/v/p>\r\n");
    }
    if(focobj == foc1) {
        cmod_set(FOC_DEV_1, cmod);
        focc_time_clr(&focc->focc1);
    } else {
        cmod_set(FOC_DEV_2, cmod);
        focc_time_clr(&focc->focc2);
    }
    return 0;
}

static int args_bemf_hdl(int argc, char **argv)
{
    UNUSED(argc);
    UNUSED(argv);
    QSH(" BEMF1_A: %f, H: %08X\r\n", focc->focc1.bemf_a);
    QSH(" BEMF1_B: %f, H: %08X\r\n", focc->focc1.bemf_b);
    QSH(" BEMF1_C: %f, H: %08X\r\n", focc->focc1.bemf_c);
    QSH(" BEMF2_A: %f, H: %08X\r\n", focc->focc2.bemf_a);
    QSH(" BEMF2_B: %f, H: %08X\r\n", focc->focc2.bemf_b);
    QSH(" BEMF2_C: %f, H: %08X\r\n", focc->focc2.bemf_c);
    return 0;
}

static int args_pwm_hdl(int argc, char **argv)
{
    FocDev dev = FOC_DEV_1;
    if(focobj == foc1) {
        dev = FOC_DEV_1;
    } else if(focobj == foc2) {
        dev = FOC_DEV_2;
    } else {
        return -1;
    }
    if(ISARG(argv[1], "1") && (argc == 3)) {
        foc_pwm_ch_set(dev, PWM_CH1, atoi(argv[2]));
    } else if(ISARG(argv[1], "2") && (argc == 3)) {
        foc_pwm_ch_set(dev, PWM_CH2, atoi(argv[2]));
    } else if(ISARG(argv[1], "3") && (argc == 3)) {
        foc_pwm_ch_set(dev, PWM_CH3, atoi(argv[2]));
    } else if(ISARG(argv[1], "start") && (argc == 2)) {
        if(focobj == foc1) {
            foc_pwm_start(FOC_DEV_1);
        } else if(focobj == foc2) {
            foc_pwm_start(FOC_DEV_2);
        } else {
            return -1;
        }
    } else if(ISARG(argv[1], "stop") && (argc == 3)) {
        if(focobj == foc1) {
            foc_pwm_stop(FOC_DEV_1);
        } else if(focobj == foc2) {
            foc_pwm_stop(FOC_DEV_2);
        } else {
            return -1;
        }
    } else {
        return -1;
    }
    return 0;
}

static int args_vbias_hdl(int argc, char **argv)
{
    UNUSED(argc);
    drv->vbus_bias = atof(argv[1]);
    return 0;
}

static int args_motor_hdl(int argc, char **argv)
{
    UNUSED(argc);
    UNUSED(argv);
    QSH(" -pole pairs:       %d\r\n", motor->poles_pairs);
    QSH(" -gear ratio:       %.3f\r\n", motor->gear_ratio);
    QSH(" -phase inductance: %.3f(mH)\r\n", motor->phase_inductance);
    QSH(" -phase resistance: %.3f(ohm)\r\n", motor->phase_resistance);
    QSH(" -rated current:    %.3f(A)\r\n", motor->rated_current);
    QSH(" -stall current:    %.3f(A)\r\n", motor->stall_current);
    QSH(" -rated voltage:    %.3f(V)\r\n", motor->rated_voltage);
    QSH(" -rated torque:     %.3f(Nm)\r\n", motor->rated_torque);
    QSH(" -stall torque:     %.3f(Nm)\r\n", motor->stall_torque);
    QSH(" -rated speed:      %.3f(rpm)\r\n", motor->rated_speed);
    return 0;
}

static int args_enc_hdl(int argc, char **argv)
{
    if(ISARG(argv[1], "clr") && argc == 2) {
        focobj->pos = 0.0f;
        focobj->epos = 0.0f;
        if(focobj == foc1) {
            enc_clr(FOC_DEV_1);
        } else {
            enc_clr(FOC_DEV_2);
        }
    } else if(argc == 1) {
        QSH(" pos: %f, epos: %f\r\n", focobj->pos, focobj->epos);
    } else {
        return -1;
    }
    return 0;
}

static int args_axis_hdl(int argc, char **argv)
{
    UNUSED(argc);
    float imax = atof(argv[1]);
    QSH(" axis calibration start ...\r\n");
    axis_calib(focobj, imax);
    QSH(" axis calibration finished\r\n");
    return 0;
}

static int args_iset_hdl(int argc, char **argv)
{
    UNUSED(argc);
    float iq = atof(argv[1]);
    qfoc_iref_set(focobj, iq, 0.0f);
    return 0;
}

static int args_pset_hdl(int argc, char **argv)
{
    UNUSED(argc);
    float p = atof(argv[1]);
    qfoc_pref_set(focobj, p);
    return 0;
}

static int args_vset_hdl(int argc, char **argv)
{
    UNUSED(argc);
    float v = atof(argv[1]);
    qfoc_vref_set(focobj, v);
    return 0;
}

static int args_ilpf_hdl(int argc, char **argv)
{
    if(argc == 1) {
        if(focobj == foc1) {
            QSH(" fc: %f\r\n", ilpf_foc1.iq.fc);
        } else {
            QSH(" fc: %f\r\n", ilpf_foc2.iq.fc);
        }
    } else if(argc == 2) {
        float fc = atof(argv[1]);
        if(fc > 0) {
            if(focobj == foc1) {
                lpf_1st_fc_set(&ilpf_foc1.iq, fc);
                lpf_1st_fc_set(&ilpf_foc1.id, fc);
            } else {
                lpf_1st_fc_set(&ilpf_foc2.iq, fc);
                lpf_1st_fc_set(&ilpf_foc2.id, fc);
            }
        } else {
            QSH(" fc must greater than 0\r\n");
        }
    } else {
        return -1;
    }
    return 0;
}

static int args_vlpf_hdl(int argc, char **argv)
{
    if(argc == 1) {
        if(focobj == foc1) {
            QSH(" fc: %f\r\n", foc1_vpinfo.vlpf.fc);
        } else {
            QSH(" fc: %f\r\n", foc2_vpinfo.vlpf.fc);
        }
    } else if(argc == 2) {
        float fc = atof(argv[1]);
        if(fc > 0) {
            if(focobj == foc1) {
                lpf_1st_fc_set(&foc1_vpinfo.vlpf, fc);
            } else {
                lpf_1st_fc_set(&foc2_vpinfo.vlpf, fc);
            }
        } else {
            QSH(" fc must greater than 0\r\n");
        }
    } else {
        return -1;
    }
    return 0;
}

NameTable foc_info_type_name[] = {
    {.id = FOC_INFO_NONE, .name = "none"},
    {.id = FOC_INFO_IABC, .name = "iabc"},
    {.id = FOC_INFO_IQD, .name = "iqd"},
    {.id = FOC_INFO_V, .name = "v"},
    {.id = FOC_INFO_P, .name = "p"},
    {.id = FOC_INFO_EDEGREE, .name = "edegree"},
};

static int args_show_hdl(int argc, char **argv)
{
    FocInfoShowType type;
    if(argc == 1) {
        QSH(" foc1 info show type: %s\r\n", foc_info_type_name[foc_info_show.foc1_type].name);
        QSH(" foc2 info show type: %s\r\n", foc_info_type_name[foc_info_show.foc2_type].name);
        return 0;
    }
    if(ISARG(argv[1], "off")) {
        type = FOC_INFO_NONE;
    } else if(ISARG(argv[1], "iqd")) {
        type = FOC_INFO_IQD;
    } else if(ISARG(argv[1], "v")) {
        type = FOC_INFO_V;
    } else if(ISARG(argv[1], "p")) {
        type = FOC_INFO_P;
    } else if(ISARG(argv[1], "edegree")) {
        type = FOC_INFO_EDEGREE;
    } else if(ISARG(argv[1], "iabc")) {
        type = FOC_INFO_IABC;
    } else {
        return -1;
    }
    if(focobj == foc1) {
        if(!foc_info_show.showing || (foc_info_show.foc1_type != FOC_INFO_NONE)) {
            foc_info_show.foc1_type = type;
            foc_info_show.showing = 1;
        } else if(type == FOC_INFO_NONE) {
            foc_info_show.showing = 0;
            foc_info_show.foc1_type = FOC_INFO_NONE;
        } else {
            QLOG_WARN("foc2 info is showing");
        }
    } else {
        if(!foc_info_show.showing || (foc_info_show.foc2_type != FOC_INFO_NONE)) {
            foc_info_show.foc2_type = type;
            foc_info_show.showing = 1;
        } else if(type == FOC_INFO_NONE) {
            foc_info_show.showing = 0;
            foc_info_show.foc2_type = FOC_INFO_NONE;
        } else {
            QLOG_WARN("foc1 info is showing");
        }
    }

    return 0;
}

static QCliArgsEntry args_foc_table[] = {
    {"ena", 2, 2, args_ena_hdl, "<0/1>"},
    {"cmod", 2, 2, args_cmod_hdl, "<idle/i/v/p>"},
    {"pwm", 2, 3, args_pwm_hdl, "[<start/stop>] [<ch> <duty>]"},
    {"vbias", 2, 2, args_vbias_hdl, "<val>"},
    {"motor", 1, 1, args_motor_hdl, "show motor parameters"},
    {"enc", 1, 2, args_enc_hdl, "<clr>"},
    {"axis", 2, 2, args_axis_hdl, "<imax>"},
    {"iset", 1, 2, args_iset_hdl, "<val>"},
    {"pset", 1, 2, args_pset_hdl, "<val>"},
    {"vset", 1, 2, args_vset_hdl, "<val>"},
    {"ilpf", 1, 2, args_ilpf_hdl, "<fc>"},
    {"vlpf", 1, 2, args_vlpf_hdl, "<fc>"},
    {"bemf", 1, 1, args_bemf_hdl, "show bemf value"},
    {"show", 1, 2, args_show_hdl, "<off/iqd/v/p/edegree/iabc>"},
};

int cmd_foc_hdl(int argc, char **argv)
{
    if(argc == 1) {
        QSH(" vbus: %f\r\n", focobj->vbus);
        QSH(" ep: %.0f, p: %f, edgree: %f, v: %f\r\n", focobj->epos, focobj->pos, focobj->edegree, focobj->vel);
        QSH(" iref: %f, vref: %f, pref: %f\r\n", focobj->iqref, focobj->vref, focobj->pref);
        QSH(" iq: %f, id: %f, ci: %f\r\n", focobj->iq, focobj->id, focobj->ci);
        if(focobj == foc1) {
            QSH(" iqmax: %f, idmax: %f, cimax: %f, velmax: %f\r\n", focc->focc1.iqmax, focc->focc1.idmax, focc->focc1.cimax, focc->focc1.vmax);
        } else {
            QSH(" iqmax: %f, idmax: %f, cimax: %f, velmax: %f\r\n", focc->focc2.iqmax, focc->focc2.idmax, focc->focc2.cimax, focc->focc2.vmax);
        }
        QSH(" ia: %f, ib: %f, ic: %f\r\n", focobj->ia, focobj->ib, focobj->ic);
        if(focobj == foc1) {
            QSH(" iamax: %f, ibmax: %f, icmax: %f\r\n", focc->focc1.iamax, focc->focc1.ibmax, focc->focc1.icmax);
        } else {
            QSH(" iamax: %f, ibmax: %f, icmax: %f\r\n", focc->focc2.iamax, focc->focc2.ibmax, focc->focc2.icmax);
        }
        QSH(" pwma: %d, pwmb: %d, pwmc: %d\r\n", focobj->pwma, focobj->pwmb, focobj->pwmc);
        QSH(" status: %d, err: %d\r\n", focobj->status, focobj->err);
        QSH(" ilimit: %f, vlimit: %f, cilimit: %f\r\n", focobj->imax, focobj->vel_max, focobj->cilimit);
        if(focobj == foc1) {
            QSH(" iloop calc us: %u, iloop us: %u, bemfloop us: %u\r\n", focc->focc1.iloop_calc_us, focc->focc1.iloop_us, focc->focc1.bemf_calc_us);
            QSH(" vloop us: %u, ploop us: %u\r\n", focc->focc1.vloop_us, focc->focc1.ploop_us);
            QSH(" cmod: %02X\r\n", focc->focc1.cmod);
        } else {
            QSH(" iloop calc us: %u, iloop us: %u, bemfloop us: %u\r\n", focc->focc2.iloop_calc_us, focc->focc2.iloop_us, focc->focc2.bemf_calc_us);
            QSH(" vloop us: %u, ploop us: %u\r\n", focc->focc2.vloop_us, focc->focc2.ploop_us);
            QSH(" cmod: %02X\r\n", focc->focc2.cmod);
        }
        return 0;
    }
    if(ISARG(argv[1], "?") && argc == 2) {
        return qterm_help(args_foc_table, sizeof(args_foc_table));
    }

    return qcli_args_handle(argc, argv, args_foc_table, sizeof(args_foc_table));
}
