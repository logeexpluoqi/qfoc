/**
 * @ Author: luoqi
 * @ Create Time: 2024-08-02 10:15
 * @ Modified by: luoqi
 * @ Modified time: 2024-10-16 15:51
 * @ Description:
 */

#ifndef _QFOC_H
#define _QFOC_H

#include <stdint.h>

typedef struct {
    uint8_t poles_pairs;
    float phase_resistance; // Ohm
    float phase_inductance; // mH

    float rated_current;    // A
    float stall_current;    // A
    float rated_torque;     // Nm
    float stall_torque;     // Nm
    float rated_voltage;    // V
    float rated_speed;      // rpm

    float kt;               // Nm/A

    float gear_ratio;
} PmsmMotor;

typedef enum {
    QFOC_DISABLE = 0,
    QFOC_ENABLE,
} QFocEnable;

typedef enum {
    QFOC_STATUS_IDLE = 0,
    QFOC_STATUS_RUNNING,
    QFOC_STATUS_ERROR,
} QFocStatus;

typedef enum {
    QFOC_ERR_NONE = 0,      // no error
    QFOC_ERR_IMAX_NOT_SET,  // over load limit
    QFOC_ERR_OIMAX,         // over current limit
    QFOC_ERR_OVMAX,         // over velocity limit
    QFOC_ERR_OPMAX,         // over position max limit
    QFOC_ERR_OPMIN,         // over position min limit
    QFOC_ERR_OVBUS,         // over vbus limit
    QFOC_ERR_OPWR,          // over pwoer limit
    QFOC_ERR_MOTOR_PARAM,   // motor parameter error
} QFocError;

typedef struct {
    QFocStatus status;
    QFocError err;
    float ep;           // degree, encoder position
    float p;            // degree, position
    float v;            // degree/s, velocity
    float iq;           // A, q-axis current
    float id;           // A, d-axis current
    float ia;           // A, phase a current
    float ib;           // A, phase b current
    float ic;           // A, phase c current
    float edegree;      // degree, electric degree, 0-360 degree
    float vbus;         // V, power supply voltage

    float pref;         // degree, position reference
    float vref;         // degree/s, velocity reference
    float iqref;        // A, iq reference
    float idref;        // A, id reference

    uint16_t pwm_max;   // pwm output max value
    uint16_t pwma;      // pwm output, phase a
    uint16_t pwmb;      // pwm output, phase b
    uint16_t pwmc;      // pwm output, phase c
    uint8_t sector;     // svm generate space vector sector, 1-6

    /* imax must over 0.0f,  vmax value is 0.0f, means no limit */
    float imax;         // A
    float vmax;         // degree/s

    /* if pmax and pmin is 0.0f, means position no limit */
    float pmax;         // degree
    float pmin;         // degree

    /* if deadzone is 0.0f, mean no deadzone */
    float deadzone;     // dead zone, unit A
    
    float vbus_max;     // V, power supply voltage max
    float i2t_limit;    // A, current to power; unit i * i * t, A^2s
    float i2t;          // W, average power in a period
    float i2t_buf;      // W, average power in a period buffer, used to calculate i2t
    uint32_t integral_times;    // i2t(power) integral 500ms times
    uint32_t integral_cnt;      // integral counter
    PmsmMotor *motor;
    
    int (*iloop_controller)(void *states, float *iq, float *id); // iq and id loop algorithm, return target iq and id
    float (*ploop_controller)(void *states); // position loop algorithm
    float (*vloop_controller)(void *states); // velocity loop algorithm
} QFoc;

/**
 * @brief: qfoc initialize
 * @param: motor, motor object, used to define a motor parameter
 * @param: pwm_max, pwm output max value
 * @param: i2t_limit, current to power limit, unit i * i * t, A^2s
 * @param: i2t_period, current to power period, unit ms
 * @param: iloop_period, iloop period, unit ms
 * @param: deadzone, deadzone, dead zone, unit A
 * @param: imax, current limit, unit A
 * @param: vmax, velocity limit, unit degree/s
 * @param: pmax, position max limit, unit degree
 * @param: pmin, position min limit, unit degree
 * @return: QFocStatus
 */
int qfoc_init(QFoc *foc, PmsmMotor *motor, uint16_t pwm_max, float vbus_max, float i2t_limit, float i2t_period, float iloop_period, float deadzone, float imax, float vmax, float pmax, float pmin);

/**
 * FOC close loop algorithm.
 * Current loop, velocity loop and postion loop control algorithm
 * need to complete by users, and regist these algorithm to this qfoc frame.
 * qfoc frame will auto-callback these algorithm in loop calculate or update
 * functions. and qfoc frame will auto put foc structure into these algorithm,
 * @param: states, foc structure by defaulte or user-defined parameters, if 
 *          using user-defined states, algo_function input states will be ignored.
 * @return: iloop_controller will output target iq value and target id value, its be
 *          used in qfoc_iloop_calc funciton.
 *          ploop_controller and vloop_controller will return position loop and velocity loop outputs,
 *          and called by qfoc_ploop_update, qfoc_vloop_update and qfoc_vploop_update
 *          function. 
 */
int qfoc_iloop_controller_set(QFoc *foc, int (*controller)(void *states, float *iq, float *id));

int qfoc_ploop_controller_set(QFoc *foc, float (*controller)(void *states));

int qfoc_vloop_controller_set(QFoc *foc, float (*controller)(void *states));

int qfoc_enable(QFoc *foc, QFocEnable ena);

/* FOC update state from vbus/current/velocity/postion sensors */
/* These update api can return foc statu, include errors */
int qfoc_vbus_update(QFoc *foc, float vbus);

int qfoc_i_update(QFoc *foc, float ia, float ib, float ic);

int qfoc_v_update(QFoc *foc, float v);

int qfoc_p_update(QFoc *foc, float p);

/* ep is encoder positon, p is a consecutive position */
/* if ep range is 0-n degree, p must not NAN, or P must be a NAN */
int qfoc_ep_update(QFoc *foc, float ep, float p);

/* FOC controller reference input set */
int qfoc_iref_set(QFoc *foc, float iqref, float idref);

int qfoc_vref_set(QFoc *foc, float vref);

int qfoc_pref_set(QFoc *foc, float pref);

/* FOC force output by given edegree, iq and id, return the svm sector */
int qfoc_force_calc(float vbus, float q, float d, float edegree, uint16_t pwm_max, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/* FOC current open-loop control */
int qfoc_oloop_calc(QFoc *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

int qfoc_iloop_calc(QFoc *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/* FOC velocity/position current double loop pid control */
/* di_limit is vloop or ploop  output limit*/
int qfoc_vloop_update(QFoc *foc, float di_limit);

int qfoc_ploop_update(QFoc *foc, float di_limit);

/* FOC position velocity pid control */
/* dv_limit is vploop  output limit*/
int qfoc_vploop_update(QFoc *foc, float dv_limit);

/* FOC phase calibration, make id aligned to A axis */
/* First call this api, and delay some time to record encoder positon, this position is the phase bias */
int qfoc_phase_calib(QFoc *foc, float idmax, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

#endif
