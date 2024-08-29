/**
 * @ Author: luoqi
 * @ Create Time: 2024-08-29 21:32
 * @ Modified by: luoqi
 * @ Modified time: 2024-08-29 21:57
 * @ Description:
 */

#ifndef _ENCODER_H
#define _ENCODER_H

#include <stdint.h>

typedef enum {
    ENC_DIR_CW,
    ENC_DIR_CCW
} EncDir;

typedef struct {
    EncDir dir;         // encoder direction
    uint32_t ppr;       // pulse per pevolution
    float epos;         // encoder absolute position
    float eorg;         // encoder origin position
    float unit;         // 1 pulse unit
    float nms;          // encoder data update interval, n * ms
    float epc;          // encoder twice epos error
    float tmc;        // t measure method counter
    uint32_t tcnt_th;   // interval time counter threshold
    uint32_t tcnt;      // interval time counter

    float p;        // encoder relative position
    float v;        // encoder velocity
} Encoder;

int enc_init(Encoder *enc, int32_t eorg, uint32_t tcnt_th, uint32_t ppr, float unit, float interval_ms, EncDir dir);

int enc_update(Encoder *enc, float epos);

int enc_clear(Encoder *enc);

int enc_zero(Encoder *enc);

#endif
