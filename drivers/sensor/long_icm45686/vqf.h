// SPDX-License-Identifier: MIT
// SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de>
// VQF: Versatile Quaternion-based Filter for IMU Orientation Estimation
//
// Modified for long-edc-framework:
//   - Default precision left undefined; includer picks via VQF_SINGLE_PRECISION
//     or VQF_MIXED_PRECISION before #include.
//   - Original defaults to VQF_MIXED_PRECISION (float compute + double filter).

#ifndef VQF_H__
#define VQF_H__

#include <stdint.h>
#include <stdbool.h>

// Precision selection — define ONE before including this header, or leave
// undefined to get the default (VQF_MIXED_PRECISION).
// #define VQF_SINGLE_PRECISION   // float everywhere
// #define VQF_MIXED_PRECISION    // float compute, double filter state (default)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Typedef for the floating-point data type used for most operations.
 *
 * By default, all floating-point calculations are performed using `double`. Set the `VQF_SINGLE_PRECISION` define to
 * change ALL types (including Butterworth filter state) to `float` — note this can cause numeric issues at high
 * sample rates due to very small filter coefficients. Set `VQF_MIXED_PRECISION` to use `float` for general
 * computations but `double` for Butterworth filter coefficients and state.
 */
#ifdef VQF_SINGLE_PRECISION
typedef float vqf_real_t;
typedef float vqf_double_t;
#elif defined(VQF_MIXED_PRECISION)
typedef float vqf_real_t;
typedef double vqf_double_t;
#else   // double precision
typedef double vqf_real_t;
typedef double vqf_double_t;
#endif

/**
 * @brief Struct containing all tuning parameters used by the VQF class.
 *
 * The parameters influence the behavior of the algorithm and are independent of the sampling rate of the IMU data. The
 * constructor sets all parameters to the default values.
 *
 * The parameters #motionBiasEstEnabled, #restBiasEstEnabled, and #magDistRejectionEnabled can be used to enable/disable
 * the main features of the VQF algorithm. The time constants #tauAcc and #tauMag can be tuned to change the trust on
 * the accelerometer and magnetometer measurements, respectively. The remaining parameters influence bias estimation
 * and magnetometer rejection.
 */
typedef struct vqf_params_s
{
    /**
     * @brief Time constant \f$\tau_\mathrm{acc}\f$ for accelerometer low-pass filtering in seconds.
     *
     * Small values for \f$\tau_\mathrm{acc}\f$ imply trust on the accelerometer measurements and while large values of
     * \f$\tau_\mathrm{acc}\f$ imply trust on the gyroscope measurements.
     *
     * The time constant \f$\tau_\mathrm{acc}\f$ corresponds to the cutoff frequency \f$f_\mathrm{c}\f$ of the
     * second-order Butterworth low-pass filter as follows: \f$f_\mathrm{c} = \frac{\sqrt{2}}{2\pi\tau_\mathrm{acc}}\f$.
     *
     * Default value: 3.0 s
     */
    vqf_real_t tauAcc;
    /**
     * @brief Time constant \f$\tau_\mathrm{mag}\f$ for magnetometer update in seconds.
     *
     * Small values for \f$\tau_\mathrm{mag}\f$ imply trust on the magnetometer measurements and while large values of
     * \f$\tau_\mathrm{mag}\f$ imply trust on the gyroscope measurements.
     *
     * The time constant \f$\tau_\mathrm{mag}\f$ corresponds to the cutoff frequency \f$f_\mathrm{c}\f$ of the
     * first-order low-pass filter for the heading correction as follows:
     * \f$f_\mathrm{c} = \frac{1}{2\pi\tau_\mathrm{mag}}\f$.
     *
     * Default value: 9.0 s
     */
    vqf_real_t tauMag;

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Enables gyroscope bias estimation during motion phases.
     *
     * If set to true (default), gyroscope bias is estimated based on the inclination correction only, i.e. without
     * using magnetometer measurements.
     */
    bool motionBiasEstEnabled;
#endif
    /**
     * @brief Enables rest detection and gyroscope bias estimation during rest phases.
     *
     * If set to true (default), phases in which the IMU is at rest are detected. During rest, the gyroscope bias
     * is estimated from the low-pass filtered gyroscope readings.
     */
    bool restBiasEstEnabled;
    /**
     * @brief Enables magnetic disturbance detection and magnetic disturbance rejection.
     *
     * If set to true (default), the magnetic field is analyzed. For short disturbed phases, the magnetometer-based
     * correction is disabled totally. If the magnetic field is always regarded as disturbed or if the duration of
     * the disturbances exceeds #magMaxRejectionTime, magnetometer-based updates are performed, but with an increased
     * time constant.
     */
    bool magDistRejectionEnabled;

    /**
     * @brief Standard deviation of the initial bias estimation uncertainty (in degrees per second).
     *
     * Default value: 0.5 °/s
     */
    vqf_real_t biasSigmaInit;
    /**
     * @brief Time in which the bias estimation uncertainty increases from 0 °/s to 0.1 °/s (in seconds).
     *
     * This value determines the system noise assumed by the Kalman filter.
     *
     * Default value: 100.0 s
     */
    vqf_real_t biasForgettingTime;
    /**
     * @brief Maximum expected gyroscope bias (in degrees per second).
     *
     * This value is used to clip the bias estimate and the measurement error in the bias estimation update step. It is
     * further used by the rest detection algorithm in order to not regard measurements with a large but constant
     * angular rate as rest.
     *
     * Default value: 2.0 °/s
     */
    vqf_real_t biasClip;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Standard deviation of the converged bias estimation uncertainty during motion (in degrees per second).
     *
     * This value determines the trust on motion bias estimation updates. A small value leads to fast convergence.
     *
     * Default value: 0.1 °/s
     */
    vqf_real_t biasSigmaMotion;
    /**
     * @brief Forgetting factor for unobservable bias in vertical direction during motion.
     *
     * As magnetometer measurements are deliberately not used during motion bias estimation, gyroscope bias is not
     * observable in vertical direction. This value is the relative weight of an artificial zero measurement that
     * ensures that the bias estimate in the unobservable direction will eventually decay to zero.
     *
     * Default value: 0.0001
     */
    vqf_real_t biasVerticalForgettingFactor;
#endif
    /**
     * @brief Standard deviation of the converged bias estimation uncertainty during rest (in degrees per second).
     *
     * This value determines the trust on rest bias estimation updates. A small value leads to fast convergence.
     *
     * Default value: 0.03 °
     */
    vqf_real_t biasSigmaRest;

    /**
     * @brief Time threshold for rest detection (in seconds).
     *
     * Rest is detected when the measurements have been close to the low-pass filtered reference for the given time.
     *
     * Default value: 1.5 s
     */
    vqf_real_t restMinT;
    /**
     * @brief Time constant for the low-pass filter used in rest detection (in seconds).
     *
     * This time constant characterizes a second-order Butterworth low-pass filter used to obtain the reference for
     * rest detection.
     *
     * Default value: 0.5 s
     */
    vqf_real_t restFilterTau;
    /**
     * @brief Angular velocity threshold for rest detection (in °/s).
     *
     * For rest to be detected, the norm of the deviation between measurement and reference must be below the given
     * threshold. (Furthermore, the absolute value of each component must be below #biasClip).
     *
     * Default value: 2.0 °/s
     */
    vqf_real_t restThGyr;
    /**
     * @brief Acceleration threshold for rest detection (in m/s²).
     *
     * For rest to be detected, the norm of the deviation between measurement and reference must be below the given
     * threshold.
     *
     * Default value: 0.5 m/s²
     */
    vqf_real_t restThAcc;

    /**
     * @brief Time constant for current norm/dip value in magnetic disturbance detection (in seconds).
     *
     * This (very fast) low-pass filter is intended to provide additional robustness when the magnetometer measurements
     * are noisy or not sampled perfectly in sync with the gyroscope measurements. Set to -1 to disable the low-pass
     * filter and directly use the magnetometer measurements.
     *
     * Default value: 0.05 s
     */
    vqf_real_t magCurrentTau;
    /**
     * @brief Time constant for the adjustment of the magnetic field reference (in seconds).
     *
     * This adjustment allows the reference estimate to converge to the observed undisturbed field.
     *
     * Default value: 20.0 s
     */
    vqf_real_t magRefTau;
    /**
     * @brief Relative threshold for the magnetic field strength for magnetic disturbance detection.
     *
     * This value is relative to the reference norm.
     *
     * Default value: 0.1 (10%)
     */
    vqf_real_t magNormTh;
    /**
     * @brief Threshold for the magnetic field dip angle for magnetic disturbance detection (in degrees).
     *
     * Default vaule: 10 °
     */
    vqf_real_t magDipTh;
    /**
     * @brief Duration after which to accept a different homogeneous magnetic field (in seconds).
     *
     * A different magnetic field reference is accepted as the new field when the measurements are within the thresholds
     * #magNormTh and #magDipTh for the given time. Additionally, only phases with sufficient movement, specified by
     * #magNewMinGyr, count.
     *
     * Default value: 20.0
     */
    vqf_real_t magNewTime;
    /**
     * @brief Duration after which to accept a homogeneous magnetic field for the first time (in seconds).
     *
     * This value is used instead of #magNewTime when there is no current estimate in order to allow for the initial
     * magnetic field reference to be obtained faster.
     *
     * Default value: 5.0
     */
    vqf_real_t magNewFirstTime;
    /**
     * @brief Minimum angular velocity needed in order to count time for new magnetic field acceptance (in °/s).
     *
     * Durations for which the angular velocity norm is below this threshold do not count towards reaching #magNewTime.
     *
     * Default value: 20.0 °/s
     */
    vqf_real_t magNewMinGyr;
    /**
     * @brief Minimum duration within thresholds after which to regard the field as undisturbed again (in seconds).
     *
     * Default value: 0.5 s
     */
    vqf_real_t magMinUndisturbedTime;
    /**
     * @brief Maximum duration of full magnetic disturbance rejection (in seconds).
     *
     * For magnetic disturbances up to this duration, heading correction is fully disabled and heading changes are
     * tracked by gyroscope only. After this duration (or for many small disturbed phases without sufficient time in the
     * undisturbed field in between), the heading correction is performed with an increased time constant (see
     * #magRejectionFactor).
     *
     * Default value: 60.0 s
     */
    vqf_real_t magMaxRejectionTime;
    /**
     * @brief Factor by which to slow the heading correction during long disturbed phases.
     *
     * After #magMaxRejectionTime of full magnetic disturbance rejection, heading correction is performed with an
     * increased time constant. This parameter (approximately) specifies the factor of the increase.
     *
     * Furthermore, after spending #magMaxRejectionTime/#magRejectionFactor seconds in an undisturbed magnetic field,
     * the time is reset and full magnetic disturbance rejection will be performed for up to #magMaxRejectionTime again.
     *
     * Default value: 2.0
     */
    vqf_real_t magRejectionFactor;
} vqf_params_t;

/**
 * @brief Struct containing the filter state of the VQF class.
 *
 * The relevant parts of the state can be accessed via functions of the VQF class, e.g. getQuat6D,
 * getQuat9D, getBiasEstimate, setBiasEstimate, getRestDetected and
 * getMagDistDetected. To reset the state to the initial values, use resetState.
 *
 * Direct access to the full state is typically not needed but can be useful in some cases, e.g. for debugging. For this
 * purpose, the state can be accessed by getState and set by setState.
 */
typedef struct vqf_state_s {
    /**
     * @brief Angular velocity strapdown integration quaternion.
     */
    vqf_real_t gyrQuat[4];
    /**
     * @brief Inclination correction quaternion.
     */
    vqf_real_t accQuat[4];
    /**
     * @brief Heading difference between inclination-corrected and full orientation.
     */
    vqf_real_t delta;
    /**
     * @brief True if it has been detected that the IMU is currently at rest.
     */
    bool restDetected;
    /**
     * @brief True if magnetic disturbances have been detected.
     */
    bool magDistDetected;
    /**
     * @brief Last low-pass filtered acceleration in the I frame.
     */
    vqf_real_t lastAccLp[3];
    /**
     * @brief Internal low-pass filter state for lastAccLp.
     */
    vqf_double_t accLpState[3*2];
    /**
     * @brief Last inclination correction angular rate.
     */
    vqf_real_t lastAccCorrAngularRate;
    /**
     * @brief Gain used for heading correction to ensure fast initial convergence.
     */
    vqf_real_t kMagInit;
    /**
     * @brief Last heading disagreement angle.
     */
    vqf_real_t lastMagDisAngle;
    /**
     * @brief Last heading correction angular rate.
     */
    vqf_real_t lastMagCorrAngularRate;
    /**
     * @brief Current gyroscope bias estimate (in rad/s).
     */
    vqf_real_t bias[3];
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    /**
     * @brief Covariance matrix of the gyroscope bias estimate (row-major 3x3).
     */
    vqf_real_t biasP[9];
#else
    vqf_real_t biasP;
#endif
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_double_t motionBiasEstRLpState[9*2];
    vqf_double_t motionBiasEstBiasLpState[2*2];
#endif
    vqf_real_t restLastSquaredDeviations[2];
    vqf_real_t restT;
    vqf_real_t restLastGyrLp[3];
    vqf_double_t restGyrLpState[3*2];
    vqf_real_t restLastAccLp[3];
    vqf_double_t restAccLpState[3*2];
    uint64_t lastGyrTsUs;
    uint64_t lastAccTsUs;
    uint64_t lastMagTsUs;
    vqf_real_t magRefNorm;
    vqf_real_t magRefDip;
    vqf_real_t magUndisturbedT;
    vqf_real_t magRejectT;
    vqf_real_t magCandidateNorm;
    vqf_real_t magCandidateDip;
    vqf_real_t magCandidateT;
    vqf_real_t magNormDip[2];
    vqf_double_t magNormDipLpState[2*2];
} vqf_state_t;

/**
 * @brief Struct containing coefficients used by the VQF class.
 */
typedef struct vqf_coeffs_s
{
    vqf_real_t gyrTs;
    vqf_real_t accTs;
    vqf_real_t magTs;
    vqf_double_t accLpB[3];
    vqf_double_t accLpA[2];
    vqf_real_t kMag;
    vqf_real_t biasP0;
    vqf_real_t biasV;
#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
    vqf_real_t biasMotionW;
    vqf_real_t biasVerticalW;
#endif
    vqf_real_t biasRestW;
    vqf_double_t restGyrLpB[3];
    vqf_double_t restGyrLpA[2];
    vqf_double_t restAccLpB[3];
    vqf_double_t restAccLpA[2];
    vqf_real_t kMagRef;
    vqf_double_t magNormDipLpB[3];
    vqf_double_t magNormDipLpA[2];
} vqf_coeffs_t;

void init_params(vqf_params_t *const params);

void initVqf(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
             vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs);

void updateGyr(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
               const vqf_real_t gyr[3]);
void updateGyrTs(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
                 const vqf_real_t gyr[3], uint64_t timestamp_us);

void updateAcc(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
               const vqf_real_t acc[3]);
void updateAccTs(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
                 const vqf_real_t acc[3], uint64_t timestamp_us);

void updateMag(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
               const vqf_real_t mag[3]);
void updateMagTs(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
                 const vqf_real_t mag[3], uint64_t timestamp_us);

void getQuat3D(vqf_state_t *const state, vqf_real_t out[4]);
void getQuat6D(vqf_state_t *const state, vqf_real_t out[4]);
void getQuat9D(vqf_state_t *const state, vqf_real_t out[4]);
vqf_real_t getDelta(vqf_state_t *const state);

vqf_real_t getBiasEstimate(vqf_state_t *const state, vqf_coeffs_t *const coeffs, vqf_real_t out[3]);
void setBiasEstimate(vqf_state_t *const state, vqf_real_t bias[3], vqf_real_t sigma);

bool getRestDetected(vqf_state_t *const state);
bool getMagDistDetected(vqf_state_t *const state);
void getRelativeRestDeviations(vqf_params_t *const params, vqf_state_t *const state, vqf_real_t out[2]);

vqf_real_t getMagRefNorm(vqf_state_t *const state);
vqf_real_t getMagRefDip(vqf_state_t *const state);
void setMagRef(vqf_state_t *const state, vqf_real_t norm, vqf_real_t dip);

void setTauAcc(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs, vqf_real_t tauAcc);
void setTauMag(vqf_params_t *const params, vqf_coeffs_t *const coeffs, vqf_real_t tauMag);

#ifndef VQF_NO_MOTION_BIAS_ESTIMATION
void setMotionBiasEstEnabled(vqf_params_t *const params, vqf_state_t *const state, bool enabled);
#endif
void setRestBiasEstEnabled(vqf_params_t *const params, vqf_state_t *const state, bool enabled);
void setMagDistRejectionEnabled(vqf_params_t *const params, vqf_state_t *const state, bool enabled);
void setRestDetectionThresholds(vqf_params_t *const params, vqf_real_t thGyr, vqf_real_t thAcc);

void resetState(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs);

#ifdef __cplusplus
}
#endif

#endif // VQF_H__
