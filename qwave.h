/**
 * @ Author: luoqi
 * @ Create Time: 2024-11-08 17:16
 * @ Modified by: luoqi
 * @ Modified time: 2025-06-19 17:44
 * @ Description:
 */

#ifndef _QWAVE_H_
#define _QWAVE_H_

#include <stdint.h>
#include <stddef.h>

#ifndef qfp_t
typedef float qfp_t;
#endif

#ifndef NAN
#define NAN (0.0f / 0.0f)
#endif

#ifndef INFINITY
#define INFINITY (1.0f / 0.0f)
#endif

typedef enum {
    QWAVE_SINE = 0,
    QWAVE_TRIANGLE,
    QWAVE_SAWTOOTH,
    QWAVE_ANTSAWTOOTH,
    QWAVE_NOISE,
    QWAVE_SQUARE
} QWaveType;

typedef struct {
    QWaveType type;       /**< Type of the waveform. */
    qfp_t fs;             /**< Sampling frequency in Hz. Must be greater than 5 times the wave frequency. */
    qfp_t frq;            /**< Wave frequency in Hz. */
    qfp_t period;         /**< Period of the waveform in seconds. */
    qfp_t half_period;    /**< Half of the period in seconds. */
    qfp_t t;              /**< Current time or phase of the waveform. */
    qfp_t ts;             /**< Time step or sampling interval. */
    qfp_t bias;           /**< Bias or offset of the waveform. */
    qfp_t amp;            /**< Amplitude of the waveform. */
    qfp_t output;         /**< Current output value of the waveform. */
    uint32_t seed;        /**< Seed value for the pseudo-random number generator. */
    uint32_t prng_state;  /**< State of the pseudo-random number generator. */
} QWaveGen;

/**
 * @brief Initialize a QWaveGen structure with the given parameters, and set the initial output value.
 * @param gen: pointer to the QWaveGen structure to be initialized.
 * @param type: type of the waveform (QWaveType).
 * @param fs: sampling frequency in Hz.
 * @param frq: wave frequency in Hz.
 * @param bias: bias or offset of the waveform.
 * @param seed: seed value for the pseudo-random number generator.
 * @return 0 on success, non-zero on failure.
 */
int qwave_init(QWaveGen *gen, QWaveType type, qfp_t fs, qfp_t frq, qfp_t bias, uint32_t seed);

/**
 * @brief Generate the next output value of the waveform based on the current state.
 * @param gen: pointer to the QWaveGen structure.
 * @return The next output value of the waveform.
 * @note This function updates the internal state of the waveform generator and returns the next output value,
 *       sampling frequency is depending on parameter fs.
 */
qfp_t qwave_gen(QWaveGen *gen);

/**
 * @brief Set the bias or offset of the waveform.
 * @param gen: pointer to the QWaveGen structure.
 * @param bias: new bias or offset value.
 * @return 0 on success, non-zero on failure.
 */
int qwave_bias_set(QWaveGen *gen, qfp_t bias);

/**
 * @brief Set the sampling frequency of the waveform.
 * @param gen: pointer to the QWaveGen structure.
 * @param fs: new sampling frequency in Hz.
 * @return 0 on success, non-zero on failure.
 */
int qwave_fs_set(QWaveGen *gen, qfp_t fs);

/**
 * @brief Set the wave frequency of the waveform.
 * @param gen: pointer to the QWaveGen structure.
 * @param frq: new wave frequency in Hz.
 * @return 0 on success, non-zero on failure.
 */
int qwave_frq_set(QWaveGen *gen, qfp_t frq);

/**
 * @brief Set the amplitude of the waveform.
 * @param gen: pointer to the QWaveGen structure.
 * @param amp: new amplitude value.
 * @return 0 on success, non-zero on failure.
 */
int qwave_amp_set(QWaveGen *gen, qfp_t amp);

/**
 * @brief Set the type of the waveform.
 * @param gen: pointer to the QWaveGen structure.
 * @param type: new type of the waveform (QWaveType).
 * @return 0 on success, non-zero on failure.
 */
int qwave_signal_set(QWaveGen *gen, QWaveType type);

/**
 * @brief Clear the QWaveGen structure, resetting it to its initial state.
 * @param gen: pointer to the QWaveGen structure.
 * @return 0 on success, non-zero on failure.
 */
int qwave_reset(QWaveGen *gen);

#endif
