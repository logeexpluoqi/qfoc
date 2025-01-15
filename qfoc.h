/**
 * @ Author: luoqi
 * @ Create Time: 2024-08-02 10:15
 * @ Modified by: luoqi
 * @ Modified time: 2025-01-07 14:51
 * @ Description:
 */

#ifndef _QFOC_H
#define _QFOC_H

#include <stdint.h>

/** 
   define qfoc float data type, 
   user can redefine thier own float type 
   to guaranty precision 
 */
#ifndef qfp_t
typedef float qfp_t;
#endif

typedef struct {
    uint8_t poles_pairs;
    qfp_t phase_resistance; // Ohm
    qfp_t phase_inductance; // mH

    qfp_t rated_current;    // A
    qfp_t stall_current;    // A
    qfp_t rated_torque;     // Nm
    qfp_t stall_torque;     // Nm
    qfp_t rated_voltage;    // V
    qfp_t rated_speed;      // rpm

    qfp_t kt;               // Nm/A

    qfp_t gear_ratio;
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
    /* track error, using in controller loop, set by controller designer */
    QFOC_ERR_OITRACK,       // over current track limit
    QFOC_ERR_OVTRACK,       // over velocity track limit
    QFOC_ERR_OPTRACK,       // over position track limit
    QFOC_ERR_PHASE,         // phase error
} QFocError;

typedef struct _qfoc{
    QFocStatus status;
    QFocError err;
    qfp_t epos;         // degree, encoder position
    qfp_t pos;          // degree, position, pos = epos * gear_ratio
    qfp_t vel;          // degree/s, velocity
    qfp_t iq;           // A, q-axis current
    qfp_t id;           // A, d-axis current
    qfp_t ia;           // A, phase a current
    qfp_t ib;           // A, phase b current
    qfp_t ic;           // A, phase c current
    qfp_t edegree;      // degree, electric degree, 0-360 degree
    qfp_t vbus;         // V, power supply voltage

    qfp_t pref;         // degree, position reference
    qfp_t vref;         // degree/s, velocity reference
    qfp_t iqref;        // A, iq reference
    qfp_t idref;        // A, id reference
    qfp_t vq;           // V, q axis voltage
    qfp_t vd;           // V, d axis voltage

    uint16_t pwm_max;   // pwm output max value
    uint16_t pwma;      // pwm output, phase a
    uint16_t pwmb;      // pwm output, phase b
    uint16_t pwmc;      // pwm output, phase c
    uint8_t sector;     // svm generate space vector sector, 1-6

    /* imax must over 0.0f,  vmax value is 0.0f, means no limit */
    qfp_t imax;         // A, q axis current limit
    qfp_t iphase_max;   // A, phase current limit
    qfp_t vel_max;         // degree/s, velocity limit

    /* if pmax and pmin is 0.0f, means position no limit */
    qfp_t pos_max;         // degree
    qfp_t pos_min;         // degree

    qfp_t vbus_max;     // V, power supply voltage max
    qfp_t cilimit;      // A, continuous current to power limit
    qfp_t ci;           // A, continuous current
    qfp_t ipower;       // average power in a period buffer, ipower += sqrt(iq * iq + id * id), in a iloop period
    uint32_t i2t_times;    // i2t(power) integral times
    uint32_t i2t_cnt;      // integral counter
    PmsmMotor *motor;
    
    int (*iloop_controller)(struct _qfoc *foc, qfp_t *vq, qfp_t *vd); // iq and id loop algorithm, return target vq and vd
    qfp_t (*ploop_controller)(struct _qfoc *foc); // position loop algorithm
    qfp_t (*vloop_controller)(struct _qfoc *foc); // velocity loop algorithm
} QFoc;

#define QFOC_NO_LIMIT   0

/**
 * @brief: qfoc initialize
 * @param: motor, motor object, used to define a motor parameter
 * @param: pwm_max, pwm output max value
 * @param: cilimit, continuous current limit in i2t_times, unit A
 * @param: i2t_times, current to power calculate times in iloop 
 * @param: imax, current limit, unit A
 * @param: vmax, velocity limit, unit degree/s
 * @param: pmax, position max limit, unit degree
 * @param: pmin, position min limit, unit degree
 * @return: QFocStatus
 */
int qfoc_init(QFoc *foc, PmsmMotor *motor, uint16_t pwm_max, qfp_t vbus_max, qfp_t cilimit, uint32_t i2t_times, qfp_t imax, qfp_t vel_max, qfp_t pos_max, qfp_t pos_min);

/**
 * FOC close loop algorithm.
 * Current loop, velocity loop and postion loop control algorithm
 * need to complete by users, and regist these algorithm to this qfoc frame.
 * qfoc frame will auto-callback these algorithm in loop calculate or update
 * functions. and qfoc frame will auto put foc structure into these algorithm,
 * @param: foc, foc structure, controller algorithm will use this structure to get foc parameters and set foc outputs.
 * @return: iloop_controller will output target iq value and target id value, its be
 *          used in qfoc_iloop_calc funciton.
 *          ploop_controller and vloop_controller will return position loop and velocity loop outputs,
 *          and called by qfoc_ploop_update, qfoc_vloop_update and qfoc_vploop_update
 *          function. 
 */
int qfoc_iloop_controller_set(QFoc *foc, int (*controller)(QFoc *foc, qfp_t *vq, qfp_t *vd));

int qfoc_ploop_controller_set(QFoc *foc, qfp_t (*controller)(QFoc *foc));

int qfoc_vloop_controller_set(QFoc *foc, qfp_t (*controller)(QFoc *foc));

int qfoc_enable(QFoc *foc, QFocEnable ena);

/* FOC update state from vbus/current/velocity/postion sensors */
/* These update api can return foc statu, include errors */
int qfoc_vbus_update(QFoc *foc, qfp_t vbus);

int qfoc_i_update(QFoc *foc, qfp_t ia, qfp_t ib, qfp_t ic);

int qfoc_vel_update(QFoc *foc, qfp_t vel);

/* ep is encoder positon update */
int qfoc_epos_update(QFoc *foc, qfp_t epos);

/* FOC controller reference input set */
int qfoc_vqd_set(QFoc *foc, qfp_t vq, qfp_t vd);

int qfoc_iref_set(QFoc *foc, qfp_t iqref, qfp_t idref);

int qfoc_vref_set(QFoc *foc, qfp_t vref);

int qfoc_pref_set(QFoc *foc, qfp_t pref);

/* FOC force output by given edegree, iq and id, return the svm sector */
int qfoc_force_calc(qfp_t vbus, qfp_t vq, qfp_t vd, qfp_t edegree, uint16_t pwm_max, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/* FOC current open-loop control */
int qfoc_oloop_calc(QFoc *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

int qfoc_iloop_calc(QFoc *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/* FOC velocity/position current double loop pid control */
/* dmax is vloop or ploop  output limit*/
int qfoc_vloop_update(QFoc *foc, qfp_t dmax);

int qfoc_ovloop_update(QFoc *foc, qfp_t dmax);

int qfoc_ploop_update(QFoc *foc, qfp_t dmax);

/* FOC position velocity pid control */
/* dmax is vploop  output limit*/
int qfoc_vploop_update(QFoc *foc, qfp_t dmax);

/* FOC phase calibration, make d axis aligned to A axis */
/* First call this api, and delay some time to record encoder positon, this position is the phase bias */
/* Its also can be used in encoder caliberation */
int qfoc_calib_calc(QFoc *foc, qfp_t vdmax, qfp_t pref, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

#endif
