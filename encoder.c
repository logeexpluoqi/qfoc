/**
 * @ Author: luoqi
 * @ Create Time: 2024-08-29 21:32
 * @ Modified by: luoqi
 * @ Modified time: 2024-08-29 21:56
 * @ Description:
 */

#include "encoder.h"

int enc_init(Encoder *enc, int32_t eorg, uint32_t tcnt_th, uint32_t ppr, float unit, float interval_ms, EncDir dir)
{
    enc->ppr = ppr;
    enc->eorg = eorg;
    enc->unit = unit;
    enc->nms = interval_ms;
    enc->dir = dir;
    enc->tcnt_th = tcnt_th;
    return 0;
}

int enc_zero(Encoder *enc)
{
    enc->eorg = 0;
    return 0;
}

int enc_update(Encoder *enc, float epos)
{
    float abs_epc;
    if(enc->dir == ENC_DIR_CCW) {
        epos = -epos;
    }
    enc->epc = epos - enc->epos;
    abs_epc = enc->epc > 0.0f ? enc->epc : -enc->epc;
    enc->epos = epos;
    enc->p = (epos - enc->eorg) * enc->unit;
    if(abs_epc < (0.001f * enc->nms * enc->ppr)) {
        enc->tcnt++;
        enc->tmc += enc->epc;
        if(enc->tcnt > enc->tcnt_th) {
            enc->tcnt = 0;
            enc->v = 1000.0f * ((float)enc->tmc * enc->unit) / (enc->nms * enc->tcnt_th);
            enc->tmc = 0;
        }
    } else {
        enc->v = 1000.0f * (enc->epc * enc->unit) / enc->nms;
        enc->tcnt = 0;
        enc->tmc = 0;
    }
    return 0;
}

int enc_clear(Encoder *enc)
{
    enc->epos = 0.0f;
    enc->eorg = 0.0f;
    enc->epc = 0.0f;
    enc->tcnt = 0;
    enc->tmc = 0.0f;
    enc->p = 0.0f;
    enc->v = 0.0f;
    return 0;
}

int enc_zero(Encoder *enc)
{
    enc->eorg = enc->epos;
    return 0;
}
