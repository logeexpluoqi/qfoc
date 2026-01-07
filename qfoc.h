/**
 * Author: luoqi
 * Created Date: 2024-08-02 10:15:21
 * Last Modified: 2026-01-07 17:31:11
 * Modified By: luoqi at <**@****>
 * Copyright (c) 2025 <*****>
 * Description:
 */

#ifndef _QFOC_H_
#define _QFOC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/** 
   define qfoc float data type, 
   user can redefine thier own float type 
   to guaranty precision 
 */
#ifndef qfp_t
typedef float qfp_t;
#endif

#ifndef NAN
#define NAN (0.0f / 0.0f)
#endif

/**
 * @brief: Structure defining the parameters of a PMSM motor.
 * Includes pole pairs, phase resistance, inductance, rated values, and gear ratio.
 */
typedef struct {
    uint8_t poles_pairs;          // Number of pole pairs
    qfp_t phase_resistance;       // Phase resistance in Ohms
    qfp_t phase_inductance;       // Phase inductance in mH

    qfp_t rated_current;          // Rated current in Amperes
    qfp_t stall_current;          // Stall current in Amperes
    qfp_t rated_torque;           // Rated torque in Nm
    qfp_t stall_torque;           // Stall torque in Nm
    qfp_t rated_voltage;          // Rated voltage in Volts
    qfp_t rated_speed;            // Rated speed in RPM

    qfp_t kt;                     // Torque constant in Nm/A
    qfp_t gear_ratio;             // Gear ratio
} PmsmMotor;

/**
 * @brief: Enum defining the enable states for FOC control.
 */
typedef enum {
    QFOC_DISABLE = 0,             // Disable FOC
    QFOC_ENABLE = 1,                  // Enable FOC
} QFocEnable;

/**
 * @brief: Enum defining the status of the FOC system.
 */
typedef enum {
    QFOC_STATUS_IDLE = 0,         // Idle state
    QFOC_STATUS_RUNNING,          // Running state
    QFOC_STATUS_ERROR,            // Error state
} QFocStatus;

/**
 * @brief: Enum defining possible errors in the FOC system.
 */
typedef enum {
    QFOC_ERR_NONE = 0,                  // No error
    QFOC_ERR_OBJ_NOT_FOUND = -1,        // FOC objects not found
    QFOC_ERR_IMAX_NOT_SET = -2,         // Maximum current limit not set
    QFOC_ERR_OIMAX = -3,                // Over current limit error
    QFOC_ERR_OVMAX = -4,                // Over velocity limit error
    QFOC_ERR_OPMAX = -5,                // Over position max limit error
    QFOC_ERR_OPMIN = -6,                // Over position min limit error
    QFOC_ERR_OVBUS = -7,                // Over vbus limit error
    QFOC_ERR_OPWR = -8,                 // Over power limit error
    QFOC_ERR_MOTOR_PARAM = -9,          // Motor parameter error
    QFOC_ERR_OITRACK = -10,             // Over current tracking limit error
    QFOC_ERR_OVTRACK = -11,             // Over velocity tracking limit error
    QFOC_ERR_OPTRACK = -12,             // Over position tracking limit error
    QFOC_ERR_PHASE = -13,               // Phase error
    QFOC_ERR_LOOP_OUTPUT = -14,         // Loop output error
} QFocError;

/**
 * @brief: Enum defining the output target for FOC calculations
 * Used to determine the next processing stage in the FOC control chain
 */
typedef enum {
    QFOC_OUT_TO_NONE = 0, // No output
    QFOC_OUT_TO_OLOOP,    // Output to open loop update stage
    QFOC_OUT_TO_ILOOP,    // Output to i loop control stage
    QFOC_OUT_TO_VLOOP,    // Output to v loop control stage
} QFocOutputTo;

/**
 * @brief: Structure holding FOC control loop parameters and references
 * Contains all the necessary reference values and intermediate results
 * for FOC control calculations across different control loops
 */
typedef struct {
    qfp_t iqref;          // Reference current for q-axis in Amperes
    qfp_t idref;          // Reference current for d-axis in Amperes
    qfp_t vref;           // Reference velocity in degrees/second
    qfp_t pref;           // Reference position in degrees
    qfp_t vq;             // Calculated voltage for q-axis in Volts
    qfp_t vd;             // Calculated voltage for d-axis in Volts
    QFocOutputTo to;      // Specifies the next processing stage in control chain
} QFocOutput;

/**
 * @brief: Main structure representing the FOC object.
 * Contains all parameters, references, outputs, and controllers for FOC operation.
 */
typedef struct qfoc QFocObj;

struct qfoc {
    QFocStatus status;            // Current status of the FOC system
    QFocError err;                // Error code of the FOC system
    qfp_t epos;                   // Encoder position in degrees
    qfp_t pos;                    // Position in degrees (epos * gear_ratio)
    qfp_t vel;                    // Velocity in degrees/second
    qfp_t iq;                     // Current for q-axis in Amperes
    qfp_t id;                     // Current for d-axis in Amperes
    qfp_t ia;                     // Phase A current in Amperes
    qfp_t ib;                     // Phase B current in Amperes
    qfp_t ic;                     // Phase C current in Amperes
    qfp_t edegree;                // Electrical degree (0-360 degrees)
    qfp_t vbus;                   // Power supply voltage in Volts

    qfp_t pref;                   // Reference position in degrees
    qfp_t vref;                   // Reference velocity in degrees/second
    qfp_t iqref;                  // Reference current for q-axis in Amperes
    qfp_t idref;                  // Reference current for d-axis in Amperes
    qfp_t vq;                     // Voltage for q-axis in Volts
    qfp_t vd;                     // Voltage for d-axis in Volts

    uint16_t pwm_max;             // Maximum PWM output value
    uint16_t pwma;                // PWM output for phase A
    uint16_t pwmb;                // PWM output for phase B
    uint16_t pwmc;                // PWM output for phase C
    uint8_t sector;               // SVM space vector sector (1-6)

    qfp_t phase_cur_max;          // Phase current limit in Amperes
    qfp_t vel_max;                // Velocity limit in degrees/second
    qfp_t pos_max;                // Maximum position limit in degrees
    qfp_t pos_min;                // Minimum position limit in degrees
    qfp_t vbus_max;               // Maximum dc supply voltage in Volts
    qfp_t cc_max;                 // Continuous current limit for i2t calculation in Amperes
    qfp_t cc_integ;               // Continuous current integral in Amperes
    qfp_t cc;                     // Continuous current in Amperes
    qfp_t ap_max;                 // Average power limit in Watts
    qfp_t ap_integ;               // Average power integral over a period
    qfp_t ap;                     // Average power over a period
    qfp_t ibus;                   // dc current in Amperes
    uint32_t samp_time;           // Sample time period in iloop times
    uint32_t samp_cnt;            // Sample counter
    PmsmMotor *motor;             // Pointer to the motor object

    QFocError (*iloopc)(const QFocObj *foc, QFocOutput *output); // Current loop controller
    QFocError (*vloopc)(const QFocObj *foc, QFocOutput *output); // Velocity loop controller
    QFocError (*ploopc)(const QFocObj *foc, QFocOutput *output); // Position loop controller
};

#define QFOC_UNLIMITED   (0)

typedef QFocError (*QFocController)(const QFocObj *foc, QFocOutput *output);

/**
 * @brief: Initializes the FOC object with motor parameters and limits.
 * @param foc: Pointer to the FOC object.
 * @param motor: Pointer to the motor object.
 * @param samptime: Sample time period in iloop times for cc, ap calc.
 * @param pwm_max: Maximum PWM output value.
 * @param vbus_max: Maximum ibus supply voltage in Volts.
 * @param cc_max: Continuous current limit in Amperes.
 * @param ap_max: Average power limit in Watts.
 * @param cur_max: Current limit for q-axis in Amperes.
 * @param vel_max: Velocity limit in degrees/second.
 * @param pos_max: Maximum position limit in degrees.
 * @param pos_min: Minimum position limit in degrees.
 * @return: Status of initialization.
 */
int qfoc_init(QFocObj *foc, PmsmMotor *motor, uint32_t samptime, uint16_t pwm_max, qfp_t vbus_max, qfp_t cc_max, qfp_t ap_max, qfp_t phase_cur_max, qfp_t vel_max, qfp_t pos_max, qfp_t pos_min);


/**
 * @brief: Sets the controllers for the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param iloop: Function pointer for the current loop controller.
 * @param vloop: Function pointer for the velocity loop controller.
 * @param ploop: Function pointer for the position loop controller.
 * @return: 0 on success, -1 on error.
 */
int qfoc_controller_set(QFocObj *foc, QFocController iloop, QFocController vloop, QFocController ploop);

/**
 * @brief: Enables or disables the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param ena: Enable state (qfoc_ena or QFOC_DISABLE).
 * @return: 0 on success, -1 on failed.
 */
int qfoc_ena(QFocObj *foc, QFocEnable ena);

/**
 * @brief: Updates the FOC system with the latest vbus voltage value.
 * @param foc: Pointer to the FOC object.
 * @param vbus: Latest vbus voltage value.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_vbus_update(QFocObj *foc, qfp_t vbus);

/**
 * @brief: Updates the FOC system with the latest phase currents.
 * @param foc: Pointer to the FOC object.
 * @param ia: Phase A current.
 * @param ib: Phase B current.
 * @param ic: Phase C current.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_iabc_update(QFocObj *foc, qfp_t ia, qfp_t ib, qfp_t ic);

/**
 * @brief: Updates the FOC system with the latest velocity value.
 * @param foc: Pointer to the FOC object.
 * @param vel: Latest velocity value.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_vel_update(QFocObj *foc, qfp_t vel);

/**
 * @brief: Updates the FOC system with the latest encoder position.
 * @param foc: Pointer to the FOC object.
 * @param epos: Latest encoder position.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_epos_update(QFocObj *foc, qfp_t epos);

/**
 * @brief: Sets the target voltages for the q-axis and d-axis in the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param vq: Target voltage for the q-axis.
 * @param vd: Target voltage for the d-axis.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_vqd_set(QFocObj *foc, qfp_t vq, qfp_t vd);

/**
 * @brief: Sets the reference currents for the q-axis and d-axis in the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param iqref: Reference current for the q-axis.
 * @param idref: Reference current for the d-axis.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_iref_set(QFocObj *foc, qfp_t iqref, qfp_t idref);

/**
 * @brief: Sets the reference velocity for the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param vref: Reference velocity.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_vref_set(QFocObj *foc, qfp_t vref);

/**
 * @brief: Sets the reference position for the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param pref: Reference position.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_pref_set(QFocObj *foc, qfp_t pref);

/**
 * @brief: Calculates the SVM sector and PWM outputs for a given electrical degree and voltages.
 * @param vbus: Bus voltage.
 * @param vq: Voltage for the q-axis.
 * @param vd: Voltage for the d-axis.
 * @param edegree: Electrical degree.
 * @param pwm_max: Maximum PWM output value.
 * @param pwma: Pointer to store PWM output for phase A.
 * @param pwmb: Pointer to store PWM output for phase B.
 * @param pwmc: Pointer to store PWM output for phase C.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_compel_exec(qfp_t vbus, qfp_t vq, qfp_t vd, qfp_t edegree, uint16_t pwm_max, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/**
 * @brief: Refreshes the FOC system in open-loop mode.
 * @param foc: Pointer to the FOC object.
 * @param pwma: Pointer to store PWM output for phase A.
 * @param pwmb: Pointer to store PWM output for phase B.
 * @param pwmc: Pointer to store PWM output for phase C.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_oloop_exec(QFocObj *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/**
 * @brief: Refreshes the FOC system in current loop mode.
 * @param foc: Pointer to the FOC object.
 * @param pwma: Pointer to store PWM output for phase A.
 * @param pwmb: Pointer to store PWM output for phase B.
 * @param pwmc: Pointer to store PWM output for phase C.
 * @return: 0 on success, -1 on failed.
 */
int qfoc_iloop_exec(QFocObj *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/**
 * @brief: Executes the velocity loop control logic.
 * @param foc: Pointer to the FOC object.
 * @return: 0 on success, -1 on error.
 */
int qfoc_vloop_exec(QFocObj *foc);

/**
 * @brief: Executes the position loop control logic.
 * @param foc: Pointer to the FOC object.
 * @return: 0 on success, -1 on error.
 */
int qfoc_ploop_exec(QFocObj *foc);

/**
 * @brief: Aligns the axis of the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param vdmax: Maximum d-axis voltage.
 * @param pref: Reference position.
 * @param pwma: Pointer to store PWM output for phase A.
 * @param pwmb: Pointer to store PWM output for phase B.
 * @param pwmc: Pointer to store PWM output for phase C.
 * @return: 0 on success, -1 on error.
 */
int qfoc_axis_align(QFocObj *foc, qfp_t vdmax, qfp_t pref, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

#ifdef __cplusplus
}
#endif

#endif
