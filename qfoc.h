/**
 * @ Author: luoqi
 * @ Create Time: 2024-08-02 10:15
 * @ Modified by: luoqi
 * @ Modified time: 2025-03-05 23:18
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

#ifndef QNULL
#define QNULL ((void *)0)
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
    QFOC_ENABLE,                  // Enable FOC
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
    QFOC_ERR_NONE = 0,            // No error
    QFOC_ERR_IMAX_NOT_SET,        // Maximum current limit not set
    QFOC_ERR_OIMAX,               // Over current limit error
    QFOC_ERR_OVMAX,               // Over velocity limit error
    QFOC_ERR_OPMAX,               // Over position max limit error
    QFOC_ERR_OPMIN,               // Over position min limit error
    QFOC_ERR_OVBUS,               // Over vbus limit error
    QFOC_ERR_OPWR,                // Over power limit error
    QFOC_ERR_MOTOR_PARAM,         // Motor parameter error
    QFOC_ERR_OITRACK,             // Over current tracking limit error
    QFOC_ERR_OVTRACK,             // Over velocity tracking limit error
    QFOC_ERR_OPTRACK,             // Over position tracking limit error
    QFOC_ERR_PHASE,               // Phase error
} QFocError;

/**
 * @brief: Structure defining the parameters used in FOC control loops.
 */
typedef struct {
    qfp_t iqref;                  // Reference current for q-axis in Amperes
    qfp_t idref;                  // Reference current for d-axis in Amperes
    qfp_t vref;                   // Reference velocity in degrees/second
    qfp_t pref;                   // Reference position in degrees
    qfp_t vq;                     // Voltage for q-axis in Volts
    qfp_t vd;                     // Voltage for d-axis in Volts
} QFocParams;

/**
 * @brief: Main structure representing the FOC object.
 * Contains all parameters, references, outputs, and controllers for FOC operation.
 */
typedef struct _qfoc {
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

    qfp_t imax;                   // Current limit for q-axis and d-axis in Amperes
    qfp_t iphase_max;             // Phase current limit in Amperes
    qfp_t vel_max;                // Velocity limit in degrees/second
    qfp_t pos_max;                // Maximum position limit in degrees
    qfp_t pos_min;                // Minimum position limit in degrees
    qfp_t vbus_max;               // Maximum power supply voltage in Volts
    qfp_t cilimit;                // Continuous current limit for i2t calculation in Amperes
    qfp_t ci;                     // Continuous current in Amperes
    qfp_t ipower;                 // Average power over a period
    uint32_t i2t_times;           // i2t integral times
    uint32_t i2t_cnt;             // Integral counter
    PmsmMotor *motor;             // Pointer to the motor object

    QFocError (*iloop_controller)(const struct _qfoc *foc, QFocParams *output); // Current loop controller
    QFocError (*ploop_controller)(const struct _qfoc *foc, QFocParams *output); // Position loop controller
    QFocError (*vloop_controller)(const struct _qfoc *foc, QFocParams *output); // Velocity loop controller
} QFocObj;

#define QFOC_NO_LIMIT   0

/**
 * @brief: Initializes the FOC object with motor parameters and limits.
 * @param foc: Pointer to the FOC object.
 * @param motor: Pointer to the motor object.
 * @param pwm_max: Maximum PWM output value.
 * @param vbus_max: Maximum power supply voltage in Volts.
 * @param cilimit: Continuous current limit for i2t calculation in Amperes.
 * @param i2t_times: Number of i2t integral times.
 * @param imax: Current limit for q-axis in Amperes.
 * @param vel_max: Velocity limit in degrees/second.
 * @param pos_max: Maximum position limit in degrees.
 * @param pos_min: Minimum position limit in degrees.
 * @return: Status of initialization.
 */
int qfoc_init(QFocObj *foc, PmsmMotor *motor, uint16_t pwm_max, qfp_t vbus_max, qfp_t cilimit, uint32_t i2t_times, qfp_t imax, qfp_t vel_max, qfp_t pos_max, qfp_t pos_min);

/**
 * @brief Sets the current loop controller for FOC.
 * Users need to implement the current loop algorithm and register it using this function.
 * The registered controller will be automatically called during the FOC loop calculations.
 *
 * @param foc Pointer to the FOC object.
 * @param controller Function pointer to the user-defined current loop controller.
 *                   The controller should calculate target iq and id values based on the FOC parameters.
 * @return 0 on success, error code otherwise.
 */
int qfoc_iloop_controller_set(QFocObj *foc, QFocError (*controller)(const QFocObj *foc, QFocParams *output));

/**
 * @brief: Sets the position loop controller for FOC.
 * Users need to implement the position loop algorithm and register it using this function.
 * The registered controller will be automatically called during the FOC loop calculations.
 * @param foc: Pointer to the FOC object.
 * @param controller: Function pointer to the user-defined position loop controller.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_ploop_controller_set(QFocObj *foc, QFocError (*controller)(const QFocObj *foc, QFocParams *output));

/**
 * @brief: Sets the velocity loop controller for FOC.
 * Users need to implement the velocity loop algorithm and register it using this function.
 * The registered controller will be automatically called during the FOC loop calculations.
 * @param foc: Pointer to the FOC object.
 * @param controller: Function pointer to the user-defined velocity loop controller.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_vloop_controller_set(QFocObj *foc, QFocError (*controller)(const QFocObj *foc, QFocParams *output));

/**
 * @brief: Enables or disables the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param ena: Enable state (QFOC_ENABLE or QFOC_DISABLE).
 * @return: 0 on success, error code otherwise.
 */
int qfoc_enable(QFocObj *foc, QFocEnable ena);

/**
 * @brief: Updates the FOC system with the latest vbus voltage value.
 * @param foc: Pointer to the FOC object.
 * @param vbus: Latest vbus voltage value.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_vbus_set(QFocObj *foc, qfp_t vbus);

/**
 * @brief: Updates the FOC system with the latest phase currents.
 * @param foc: Pointer to the FOC object.
 * @param ia: Phase A current.
 * @param ib: Phase B current.
 * @param ic: Phase C current.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_i_set(QFocObj *foc, qfp_t ia, qfp_t ib, qfp_t ic);

/**
 * @brief: Updates the FOC system with the latest velocity value.
 * @param foc: Pointer to the FOC object.
 * @param vel: Latest velocity value.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_v_set(QFocObj *foc, qfp_t vel);

/**
 * @brief: Updates the FOC system with the latest encoder position.
 * @param foc: Pointer to the FOC object.
 * @param epos: Latest encoder position.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_epos_set(QFocObj *foc, qfp_t epos);

/**
 * @brief: Sets the target voltages for the q-axis and d-axis in the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param vq: Target voltage for the q-axis.
 * @param vd: Target voltage for the d-axis.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_vqd_set(QFocObj *foc, qfp_t vq, qfp_t vd);

/**
 * @brief: Sets the reference currents for the q-axis and d-axis in the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param iqref: Reference current for the q-axis.
 * @param idref: Reference current for the d-axis.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_iref_set(QFocObj *foc, qfp_t iqref, qfp_t idref);

/**
 * @brief: Sets the reference velocity for the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param vref: Reference velocity.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_vref_set(QFocObj *foc, qfp_t vref);

/**
 * @brief: Sets the reference position for the FOC system.
 * @param foc: Pointer to the FOC object.
 * @param pref: Reference position.
 * @return: 0 on success, error code otherwise.
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
 * @return: 0 on success, error code otherwise.
 */
int qfoc_force_calc(qfp_t vbus, qfp_t vq, qfp_t vd, qfp_t edegree, uint16_t pwm_max, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/**
 * @brief: Updates the FOC system in open-loop mode.
 * @param foc: Pointer to the FOC object.
 * @param pwma: Pointer to store PWM output for phase A.
 * @param pwmb: Pointer to store PWM output for phase B.
 * @param pwmc: Pointer to store PWM output for phase C.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_oloop_update(QFocObj *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/**
 * @brief: Updates the FOC system in current loop mode.
 * @param foc: Pointer to the FOC object.
 * @param pwma: Pointer to store PWM output for phase A.
 * @param pwmb: Pointer to store PWM output for phase B.
 * @param pwmc: Pointer to store PWM output for phase C.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_iloop_update(QFocObj *foc, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

/**
 * @brief: Updates the FOC system in velocity loop mode.
 * @param foc: Pointer to the FOC object.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_vloop_update(QFocObj *foc);

/**
 * @brief: Updates the FOC system in position loop mode.
 * @param foc: Pointer to the FOC object.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_ploop_update(QFocObj *foc);

/**
 * @brief: Performs phase calibration to align the d-axis with phase A.
 * This function should be called first, followed by a delay to record the encoder position.
 * The recorded position represents the phase bias and can also be used for encoder calibration.
 * @param foc: Pointer to the FOC object.
 * @param vdmax: Maximum voltage for the d-axis.
 * @param pref: Reference position.
 * @param pwma: Pointer to store PWM output for phase A.
 * @param pwmb: Pointer to store PWM output for phase B.
 * @param pwmc: Pointer to store PWM output for phase C.
 * @return: 0 on success, error code otherwise.
 */
int qfoc_calib_calc(QFocObj *foc, qfp_t vdmax, qfp_t pref, uint16_t *pwma, uint16_t *pwmb, uint16_t *pwmc);

#endif
