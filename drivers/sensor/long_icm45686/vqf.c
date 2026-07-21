/* SPDX-License-Identifier: MIT */
/* SPDX-FileCopyrightText: 2024 Hugo Chiang */
/* SPDX-FileCopyrightText: 2021 Daniel Laidig <laidig@control.tu-berlin.de> */
/*
 * VQF sensor fusion implementation.
 * See vqf.h for API documentation.
 *
 * Modified for long-edc-framework:
 *   - Removed CMSIS-DSP optional acceleration (not needed at 100 Hz).
 */

#define VQF_SINGLE_PRECISION   /* all-float, safe at 100 Hz */

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "vqf.h"

/* ---------- math helpers (all float: VQF_SINGLE_PRECISION) ---------- */

#define VQF_SIN(x)     sinf(x)
#define VQF_COS(x)     cosf(x)
#define VQF_SQRT(x)    sqrtf(x)
#define VQF_ATAN2(y,x) atan2f(y, x)

#define EPS   FLT_EPSILON
#define NaN   NAN
#define M_SQRT2f    1.41421356237309504880f
#define M_PIf       3.14159265358979323846f

static inline void vqf_sin_cos(vqf_real_t angle, vqf_real_t *s, vqf_real_t *c)
{
	*c = cosf(angle);
	*s = sinf(angle);
}

static inline vqf_real_t vqf_asin(vqf_real_t x) { return asinf(x); }
static inline vqf_real_t vqf_acos(vqf_real_t x) { return acosf(x); }

/* ---------- parameter init ---------- */

void init_params(vqf_params_t *const params)
{
	params->tauAcc = 3.0f;
	params->tauMag = 9.0f;
	params->motionBiasEstEnabled = true;
	params->restBiasEstEnabled = true;
	params->magDistRejectionEnabled = true;
	params->biasSigmaInit = 0.5f;
	params->biasForgettingTime = 100.0f;
	params->biasClip = 2.0f;
	params->biasSigmaMotion = 0.1f;
	params->biasVerticalForgettingFactor = 0.0001f;
	params->biasSigmaRest = 0.03f;
	params->restMinT = 1.5f;
	params->restFilterTau = 0.5f;
	params->restThGyr = 2.0f;
	params->restThAcc = 0.5f;
	params->magCurrentTau = 0.05f;
	params->magRefTau = 20.0f;
	params->magNormTh = 0.1f;
	params->magDipTh = 10.0f;
	params->magNewTime = 20.0f;
	params->magNewFirstTime = 5.0f;
	params->magNewMinGyr = 20.0f;
	params->magMinUndisturbedTime = 0.5f;
	params->magMaxRejectionTime = 60.0f;
	params->magRejectionFactor = 2.0f;
}

/* ---------- vector/matrix utilities ---------- */

static void vqf_fill_real(vqf_real_t *dst, size_t n, vqf_real_t val)
{
	for (size_t i = 0; i < n; i++) { dst[i] = val; }
}
static void vqf_fill_double(vqf_double_t *dst, size_t n, vqf_double_t val)
{
	for (size_t i = 0; i < n; i++) { dst[i] = val; }
}
static float vqf_square(float x) { return x * x; }
static float vqf_max(float a, float b) { return a > b ? a : b; }
static float vqf_min(float a, float b) { return a < b ? a : b; }

static vqf_real_t norm(const vqf_real_t vec[], size_t N)
{
	vqf_real_t s = 0;
	for (size_t i = 0; i < N; i++) { s += vec[i] * vec[i]; }
	return VQF_SQRT(s);
}

static void normalize(vqf_real_t vec[], size_t N)
{
	vqf_real_t n = norm(vec, N);
	if (n < EPS) { return; }
	for (size_t i = 0; i < N; i++) { vec[i] /= n; }
}

static void clip(vqf_real_t vec[], size_t N, vqf_real_t min, vqf_real_t max)
{
	for (size_t i = 0; i < N; i++) {
		if (vec[i] < min) { vec[i] = min; }
		else if (vec[i] > max) { vec[i] = max; }
	}
}

/* ---------- quaternion math ---------- */

static void quatMultiply(const vqf_real_t q1[4], const vqf_real_t q2[4], vqf_real_t out[4])
{
	vqf_real_t w = q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3];
	vqf_real_t x = q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2];
	vqf_real_t y = q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1];
	vqf_real_t z = q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0];
	out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

static void quatSetToIdentity(vqf_real_t out[4])
{
	out[0] = 1; out[1] = 0; out[2] = 0; out[3] = 0;
}

static void quatApplyDelta(vqf_real_t q[4], vqf_real_t delta, vqf_real_t out[4])
{
	vqf_real_t half_delta = delta * 0.5f;
	vqf_real_t s, c;
	vqf_sin_cos(half_delta, &s, &c);
	vqf_real_t w = c * q[0] - s * q[3];
	vqf_real_t x = c * q[1] - s * q[2];
	vqf_real_t y = c * q[2] + s * q[1];
	vqf_real_t z = c * q[3] + s * q[0];
	out[0] = w; out[1] = x; out[2] = y; out[3] = z;
}

static void quatRotate(const vqf_real_t q[4], const vqf_real_t v[3], vqf_real_t out[3])
{
	vqf_real_t x = (1 - 2*q[2]*q[2] - 2*q[3]*q[3])*v[0] + 2*v[1]*(q[2]*q[1] - q[0]*q[3]) + 2*v[2]*(q[0]*q[2] + q[3]*q[1]);
	vqf_real_t y = 2*v[0]*(q[0]*q[3] + q[2]*q[1]) + v[1]*(1 - 2*q[1]*q[1] - 2*q[3]*q[3]) + 2*v[2]*(q[2]*q[3] - q[1]*q[0]);
	vqf_real_t z = 2*v[0]*(q[3]*q[1] - q[0]*q[2]) + 2*v[1]*(q[0]*q[1] + q[3]*q[2]) + v[2]*(1 - 2*q[1]*q[1] - 2*q[2]*q[2]);
	out[0] = x; out[1] = y; out[2] = z;
}

/* ---------- Butterworth filter ---------- */

static vqf_real_t gainFromTau(vqf_real_t tau, vqf_real_t Ts)
{
	if (tau < 0.0f) { return 0; }
	if (tau == 0.0f) { return 1; }
	return 1.0f - expf(-Ts / tau);
}

static void filterCoeffs(vqf_real_t tau, vqf_real_t Ts, vqf_double_t outB[3], vqf_double_t outA[2])
{
	if (tau <= 0.0f || Ts <= 0.0f || tau < Ts * 0.5f) {
		outB[0] = 1; outB[1] = 0; outB[2] = 0;
		outA[0] = 0; outA[1] = 0;
		return;
	}
	vqf_double_t fc = (M_SQRT2f / (2.0f * (vqf_double_t)M_PIf)) / (vqf_double_t)tau;
	vqf_double_t C  = tanf((vqf_double_t)M_PIf * fc * (vqf_double_t)Ts);
	vqf_double_t D  = C * C + (vqf_double_t)M_SQRT2f * C + 1.0;
	vqf_double_t b0 = C * C / D;
	outB[0] = b0;
	outB[1] = 2.0 * b0;
	outB[2] = b0;
	outA[0] = 2.0 * (C * C - 1.0) / D;
	outA[1] = (1.0 - (vqf_double_t)M_SQRT2f * C + C * C) / D;
}

static void filterInitialState(vqf_real_t x0, const vqf_double_t b[], const vqf_double_t a[], vqf_double_t out[2])
{
	out[0] = x0 * (1.0 - b[0]);
	out[1] = x0 * (b[2] - a[1]);
}

static void filterAdaptStateForCoeffChange(vqf_real_t last_y[], size_t N, const vqf_double_t b_old[3],
					   const vqf_double_t a_old[2], const vqf_double_t b_new[3],
					   const vqf_double_t a_new[2], vqf_double_t state[])
{
	if (isnan(state[0])) { return; }
	for (size_t i = 0; i < N; i++) {
		state[0+2*i] = state[0+2*i] + (b_old[0] - b_new[0]) * last_y[i];
		state[1+2*i] = state[1+2*i] + (b_old[1] - b_new[1] - a_old[0] + a_new[0]) * last_y[i];
	}
}

static vqf_real_t filterStep(vqf_real_t x, const vqf_double_t b[3], const vqf_double_t a[2], vqf_double_t state[2])
{
	vqf_double_t y = b[0] * x + state[0];
	state[0] = b[1] * x - a[0] * y + state[1];
	state[1] = b[2] * x - a[1] * y;
	return (vqf_real_t)y;
}

static void filterVec(const vqf_real_t x[], size_t N, vqf_real_t tau, vqf_real_t Ts,
		      const vqf_double_t b[3], const vqf_double_t a[2], vqf_double_t state[], vqf_real_t out[])
{
	if (isnan(state[0])) {
		if (isnan(state[1])) {
			state[1] = 0;
			for (size_t i = 0; i < N; i++) { state[2+i] = 0; }
		}
		state[1] += 1;
		for (size_t i = 0; i < N; i++) {
			state[2+i] += x[i];
			out[i] = state[2+i] / state[1];
		}
		if (state[1] * Ts >= tau) {
			for (size_t i = 0; i < N; i++) { filterInitialState(out[i], b, a, state + 2 * i); }
		}
		return;
	}
	for (size_t i = 0; i < N; i++) {
		out[i] = filterStep(x[i], b, a, state + 2 * i);
	}
}

/* ---------- 3x3 matrix operations ---------- */

static void matrix3SetToScaledIdentity(vqf_real_t scale, vqf_real_t out[9])
{
	out[0] = scale;  out[1] = 0;      out[2] = 0;
	out[3] = 0;      out[4] = scale;  out[5] = 0;
	out[6] = 0;      out[7] = 0;      out[8] = scale;
}

static void matrix3Multiply(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
	vqf_real_t tmp[9];
	tmp[0] = in1[0]*in2[0] + in1[1]*in2[3] + in1[2]*in2[6];
	tmp[1] = in1[0]*in2[1] + in1[1]*in2[4] + in1[2]*in2[7];
	tmp[2] = in1[0]*in2[2] + in1[1]*in2[5] + in1[2]*in2[8];
	tmp[3] = in1[3]*in2[0] + in1[4]*in2[3] + in1[5]*in2[6];
	tmp[4] = in1[3]*in2[1] + in1[4]*in2[4] + in1[5]*in2[7];
	tmp[5] = in1[3]*in2[2] + in1[4]*in2[5] + in1[5]*in2[8];
	tmp[6] = in1[6]*in2[0] + in1[7]*in2[3] + in1[8]*in2[6];
	tmp[7] = in1[6]*in2[1] + in1[7]*in2[4] + in1[8]*in2[7];
	tmp[8] = in1[6]*in2[2] + in1[7]*in2[5] + in1[8]*in2[8];
	memcpy(out, tmp, sizeof(tmp));
}

static void matrix3MultiplyTpsFirst(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
	vqf_real_t tmp[9];
	tmp[0] = in1[0]*in2[0] + in1[3]*in2[3] + in1[6]*in2[6];
	tmp[1] = in1[0]*in2[1] + in1[3]*in2[4] + in1[6]*in2[7];
	tmp[2] = in1[0]*in2[2] + in1[3]*in2[5] + in1[6]*in2[8];
	tmp[3] = in1[1]*in2[0] + in1[4]*in2[3] + in1[7]*in2[6];
	tmp[4] = in1[1]*in2[1] + in1[4]*in2[4] + in1[7]*in2[7];
	tmp[5] = in1[1]*in2[2] + in1[5]*in2[5] + in1[7]*in2[8];
	tmp[6] = in1[2]*in2[0] + in1[5]*in2[3] + in1[8]*in2[6];
	tmp[7] = in1[2]*in2[1] + in1[5]*in2[4] + in1[8]*in2[7];
	tmp[8] = in1[2]*in2[2] + in1[5]*in2[5] + in1[8]*in2[8];
	memcpy(out, tmp, sizeof(tmp));
}

static void matrix3MultiplyTpsSecond(const vqf_real_t in1[9], const vqf_real_t in2[9], vqf_real_t out[9])
{
	vqf_real_t tmp[9];
	tmp[0] = in1[0]*in2[0] + in1[1]*in2[1] + in1[2]*in2[2];
	tmp[1] = in1[0]*in2[3] + in1[1]*in2[4] + in1[2]*in2[5];
	tmp[2] = in1[0]*in2[6] + in1[1]*in2[7] + in1[2]*in2[8];
	tmp[3] = in1[3]*in2[0] + in1[4]*in2[1] + in1[5]*in2[2];
	tmp[4] = in1[3]*in2[3] + in1[4]*in2[4] + in1[5]*in2[5];
	tmp[5] = in1[3]*in2[6] + in1[4]*in2[7] + in1[5]*in2[8];
	tmp[6] = in1[6]*in2[0] + in1[7]*in2[1] + in1[8]*in2[2];
	tmp[7] = in1[6]*in2[3] + in1[7]*in2[4] + in1[8]*in2[5];
	tmp[8] = in1[6]*in2[6] + in1[7]*in2[7] + in1[8]*in2[8];
	memcpy(out, tmp, sizeof(tmp));
}

static bool matrix3Inv(const vqf_real_t in[9], vqf_real_t out[9])
{
	vqf_double_t A = (vqf_double_t)in[4]*in[8] - (vqf_double_t)in[5]*in[7];
	vqf_double_t D = (vqf_double_t)in[2]*in[7] - (vqf_double_t)in[1]*in[8];
	vqf_double_t G = (vqf_double_t)in[1]*in[5] - (vqf_double_t)in[2]*in[4];
	vqf_double_t B = (vqf_double_t)in[5]*in[6] - (vqf_double_t)in[3]*in[8];
	vqf_double_t E = (vqf_double_t)in[0]*in[8] - (vqf_double_t)in[2]*in[6];
	vqf_double_t H = (vqf_double_t)in[2]*in[3] - (vqf_double_t)in[0]*in[5];
	vqf_double_t C = (vqf_double_t)in[3]*in[7] - (vqf_double_t)in[4]*in[6];
	vqf_double_t F = (vqf_double_t)in[1]*in[6] - (vqf_double_t)in[0]*in[7];
	vqf_double_t I = (vqf_double_t)in[0]*in[4] - (vqf_double_t)in[1]*in[3];

	vqf_double_t det = in[0]*A + in[1]*B + in[2]*C;

	if (det >= -EPS && det <= EPS) {
		vqf_fill_real(out, 9, 0);
		return false;
	}

	out[0] = (vqf_real_t)(A / det); out[1] = (vqf_real_t)(D / det); out[2] = (vqf_real_t)(G / det);
	out[3] = (vqf_real_t)(B / det); out[4] = (vqf_real_t)(E / det); out[5] = (vqf_real_t)(H / det);
	out[6] = (vqf_real_t)(C / det); out[7] = (vqf_real_t)(F / det); out[8] = (vqf_real_t)(I / det);
	return true;
}

/* ---------- VQF update steps ---------- */

static void updateGyr_internal(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
			       vqf_real_t Ts, const vqf_real_t gyr[3])
{
	/* rest detection */
	if (params->restBiasEstEnabled || params->magDistRejectionEnabled) {
		filterVec(gyr, 3, params->restFilterTau, Ts, coeffs->restGyrLpB, coeffs->restGyrLpA,
			  state->restGyrLpState, state->restLastGyrLp);

		state->restLastSquaredDeviations[0] =
			vqf_square(gyr[0] - state->restLastGyrLp[0]) +
			vqf_square(gyr[1] - state->restLastGyrLp[1]) +
			vqf_square(gyr[2] - state->restLastGyrLp[2]);

		vqf_real_t biasClip = params->biasClip * (M_PIf / 180.0f);
		if (state->restLastSquaredDeviations[0] >= vqf_square(params->restThGyr * (M_PIf / 180.0f))
		    || fabsf(state->restLastGyrLp[0]) > biasClip
		    || fabsf(state->restLastGyrLp[1]) > biasClip
		    || fabsf(state->restLastGyrLp[2]) > biasClip) {
			state->restT = 0.0f;
			state->restDetected = false;
		}
	}

	/* remove estimated bias */
	vqf_real_t gyrNoBias[3] = { gyr[0] - state->bias[0], gyr[1] - state->bias[1], gyr[2] - state->bias[2] };

	/* gyroscope prediction step (strapdown integration) */
	vqf_real_t gyrNorm = norm(gyrNoBias, 3);
	vqf_real_t angle = gyrNorm * Ts;
	if (gyrNorm > EPS) {
		vqf_real_t s, c;
		vqf_sin_cos(angle * 0.5f, &s, &c);
		s /= gyrNorm;
		vqf_real_t gyrStepQuat[4] = { c, s * gyrNoBias[0], s * gyrNoBias[1], s * gyrNoBias[2] };
		quatMultiply(state->gyrQuat, gyrStepQuat, state->gyrQuat);
		normalize(state->gyrQuat, 4);
	}
}

void updateGyr(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs, const vqf_real_t gyr[3])
{
	updateGyr_internal(params, state, coeffs, coeffs->gyrTs, gyr);
}

void updateGyrTs(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
		 const vqf_real_t gyr[3], uint64_t timestamp_us)
{
	vqf_real_t dt = coeffs->gyrTs;
	if (state->lastGyrTsUs != 0 && timestamp_us > state->lastGyrTsUs) {
		uint64_t diff = timestamp_us - state->lastGyrTsUs;
		if (diff > 0 && diff <= 10000000ULL) { dt = (vqf_real_t)diff / 1e6f; }
	}
	updateGyr_internal(params, state, coeffs, dt, gyr);
	if (timestamp_us > state->lastGyrTsUs || state->lastGyrTsUs == 0) { state->lastGyrTsUs = timestamp_us; }
}

static void updateAcc_internal(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
			       vqf_real_t Ts, const vqf_real_t acc[3])
{
	/* ignore [0 0 0] */
	if (acc[0] == 0.0f && acc[1] == 0.0f && acc[2] == 0.0f) { return; }

	/* rest detection */
	if (params->restBiasEstEnabled) {
		filterVec(acc, 3, params->restFilterTau, Ts, coeffs->restAccLpB, coeffs->restAccLpA,
			  state->restAccLpState, state->restLastAccLp);

		state->restLastSquaredDeviations[1] =
			vqf_square(acc[0] - state->restLastAccLp[0]) +
			vqf_square(acc[1] - state->restLastAccLp[1]) +
			vqf_square(acc[2] - state->restLastAccLp[2]);

		if (state->restLastSquaredDeviations[1] >= vqf_square(params->restThAcc)) {
			state->restT = 0.0f;
			state->restDetected = false;
		} else {
			state->restT += Ts;
			if (state->restT >= params->restMinT) { state->restDetected = true; }
		}
	}

	vqf_real_t accEarth[3];

	/* filter acc in inertial frame */
	quatRotate(state->gyrQuat, acc, accEarth);
	filterVec(accEarth, 3, params->tauAcc, Ts, coeffs->accLpB, coeffs->accLpA, state->accLpState, state->lastAccLp);

	/* transform to 6D earth frame and normalize */
	quatRotate(state->accQuat, state->lastAccLp, accEarth);
	normalize(accEarth, 3);

	/* inclination correction */
	vqf_real_t accCorrQuat[4];
	vqf_real_t q_w_arg = vqf_min(vqf_max((accEarth[2] + 1.0f) * 0.5f, 0.0f), 1.0f);
	vqf_real_t q_w = VQF_SQRT(q_w_arg);
	if (q_w > 1e-4f) {
		accCorrQuat[0] = q_w;
		accCorrQuat[1] = 0.5f * accEarth[1] / q_w;
		accCorrQuat[2] = -0.5f * accEarth[0] / q_w;
		accCorrQuat[3] = 0;
	} else {
		vqf_real_t sin_angle = VQF_SQRT(accEarth[0]*accEarth[0] + accEarth[1]*accEarth[1]);
		if (sin_angle > 1e-6f) {
			vqf_real_t sin_half = VQF_SQRT(vqf_min((1.0f - accEarth[2]) * 0.5f, 1.0f));
			accCorrQuat[0] = q_w;
			accCorrQuat[1] = sin_half * accEarth[1] / sin_angle;
			accCorrQuat[2] = sin_half * (-accEarth[0]) / sin_angle;
			accCorrQuat[3] = 0;
		} else {
			accCorrQuat[0] = 0; accCorrQuat[1] = 1; accCorrQuat[2] = 0; accCorrQuat[3] = 0;
		}
	}
	quatMultiply(accCorrQuat, state->accQuat, state->accQuat);
	normalize(state->accQuat, 4);

	/* correction angular rate for debugging */
	vqf_real_t accEarthZ = vqf_min(vqf_max(accEarth[2], -1.0f), 1.0f);
	state->lastAccCorrAngularRate = vqf_acos(accEarthZ) / Ts;

	/* ---------- bias estimation (Kalman filter) ---------- */
	if (params->motionBiasEstEnabled || params->restBiasEstEnabled) {
		vqf_real_t biasClip = params->biasClip * (M_PIf / 180.0f);
		vqf_real_t accGyrQuat[4], R[9], biasLp[2];

		getQuat6D(state, accGyrQuat);
		/* rotation matrix from quaternion */
		R[0] = 1 - 2*vqf_square(accGyrQuat[2]) - 2*vqf_square(accGyrQuat[3]);
		R[1] = 2*(accGyrQuat[2]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[3]);
		R[2] = 2*(accGyrQuat[0]*accGyrQuat[2] + accGyrQuat[3]*accGyrQuat[1]);
		R[3] = 2*(accGyrQuat[0]*accGyrQuat[3] + accGyrQuat[2]*accGyrQuat[1]);
		R[4] = 1 - 2*vqf_square(accGyrQuat[1]) - 2*vqf_square(accGyrQuat[3]);
		R[5] = 2*(accGyrQuat[2]*accGyrQuat[3] - accGyrQuat[1]*accGyrQuat[0]);
		R[6] = 2*(accGyrQuat[3]*accGyrQuat[1] - accGyrQuat[0]*accGyrQuat[2]);
		R[7] = 2*(accGyrQuat[0]*accGyrQuat[1] + accGyrQuat[3]*accGyrQuat[2]);
		R[8] = 1 - 2*vqf_square(accGyrQuat[1]) - 2*vqf_square(accGyrQuat[2]);

		biasLp[0] = R[0]*state->bias[0] + R[1]*state->bias[1] + R[2]*state->bias[2];
		biasLp[1] = R[3]*state->bias[0] + R[4]*state->bias[1] + R[5]*state->bias[2];

		filterVec(R, 9, params->tauAcc, Ts, coeffs->accLpB, coeffs->accLpA, state->motionBiasEstRLpState, R);
		filterVec(biasLp, 2, params->tauAcc, Ts, coeffs->accLpB, coeffs->accLpA, state->motionBiasEstBiasLpState, biasLp);

		vqf_real_t w[3], e[3];
		if (state->restDetected && params->restBiasEstEnabled) {
			e[0] = state->restLastGyrLp[0] - state->bias[0];
			e[1] = state->restLastGyrLp[1] - state->bias[1];
			e[2] = state->restLastGyrLp[2] - state->bias[2];
			matrix3SetToScaledIdentity(1.0f, R);
			w[0] = coeffs->biasRestW; w[1] = coeffs->biasRestW; w[2] = coeffs->biasRestW;
		} else if (params->motionBiasEstEnabled) {
			e[0] = -accEarth[1]/Ts + biasLp[0] - R[0]*state->bias[0] - R[1]*state->bias[1] - R[2]*state->bias[2];
			e[1] = accEarth[0]/Ts + biasLp[1] - R[3]*state->bias[0] - R[4]*state->bias[1] - R[5]*state->bias[2];
			e[2] = -R[6]*state->bias[0] - R[7]*state->bias[1] - R[8]*state->bias[2];
			w[0] = coeffs->biasMotionW; w[1] = coeffs->biasMotionW; w[2] = coeffs->biasVerticalW;
		} else {
			w[0] = w[1] = w[2] = -1; /* disable */
		}

		/* predict P += V */
		if (state->biasP[0] < coeffs->biasP0) { state->biasP[0] += coeffs->biasV; }
		if (state->biasP[4] < coeffs->biasP0) { state->biasP[4] += coeffs->biasV; }
		if (state->biasP[8] < coeffs->biasP0) { state->biasP[8] += coeffs->biasV; }

		if (w[0] >= 0) {
			clip(e, 3, -biasClip, biasClip);

			/* K = P R^T inv(W + R P R^T) */
			vqf_real_t K[9];
			matrix3MultiplyTpsSecond(state->biasP, R, K);
			matrix3Multiply(R, K, K);
			K[0] += w[0]; K[4] += w[1]; K[8] += w[2];
			matrix3Inv(K, K);
			matrix3MultiplyTpsFirst(R, K, K);
			matrix3Multiply(state->biasP, K, K);

			/* bias += K e */
			state->bias[0] += K[0]*e[0] + K[1]*e[1] + K[2]*e[2];
			state->bias[1] += K[3]*e[0] + K[4]*e[1] + K[5]*e[2];
			state->bias[2] += K[6]*e[0] + K[7]*e[1] + K[8]*e[2];

			/* P = P - K R P */
			matrix3Multiply(K, R, K);
			matrix3Multiply(K, state->biasP, K);
			for (size_t i = 0; i < 9; i++) { state->biasP[i] -= K[i]; }

			/* enforce symmetry & clamp diagonal */
			state->biasP[1] = state->biasP[3] = 0.5f * (state->biasP[1] + state->biasP[3]);
			state->biasP[2] = state->biasP[6] = 0.5f * (state->biasP[2] + state->biasP[6]);
			state->biasP[5] = state->biasP[7] = 0.5f * (state->biasP[5] + state->biasP[7]);
			for (int i = 0; i < 3; i++) { if (state->biasP[i*4] < 0) state->biasP[i*4] = 0; }

			clip(state->bias, 3, -biasClip, biasClip);
		}
	}
}

void updateAcc(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs, const vqf_real_t acc[3])
{
	updateAcc_internal(params, state, coeffs, coeffs->accTs, acc);
}

void updateAccTs(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
		 const vqf_real_t acc[3], uint64_t timestamp_us)
{
	vqf_real_t dt = coeffs->accTs;
	if (state->lastAccTsUs != 0 && timestamp_us > state->lastAccTsUs) {
		uint64_t diff = timestamp_us - state->lastAccTsUs;
		if (diff > 0 && diff <= 10000000ULL) { dt = (vqf_real_t)diff / 1e6f; }
	}
	updateAcc_internal(params, state, coeffs, dt, acc);
	if (timestamp_us > state->lastAccTsUs || state->lastAccTsUs == 0) { state->lastAccTsUs = timestamp_us; }
}

static void updateMag_internal(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
			       vqf_real_t Ts, const vqf_real_t mag[3])
{
	if (mag[0] == 0.0f && mag[1] == 0.0f && mag[2] == 0.0f) { return; }

	vqf_real_t magEarth[3];
	vqf_real_t accGyrQuat[4];
	getQuat6D(state, accGyrQuat);
	quatRotate(accGyrQuat, mag, magEarth);

	if (params->magDistRejectionEnabled) {
		state->magNormDip[0] = norm(magEarth, 3);
		if (state->magNormDip[0] < EPS) { return; }
		vqf_real_t magSinDip = magEarth[2] / state->magNormDip[0];
		magSinDip = vqf_min(vqf_max(magSinDip, -1.0f), 1.0f);
		state->magNormDip[1] = -vqf_asin(magSinDip);

		if (params->magCurrentTau > 0) {
			filterVec(state->magNormDip, 2, params->magCurrentTau, Ts, coeffs->magNormDipLpB,
				  coeffs->magNormDipLpA, state->magNormDipLpState, state->magNormDip);
		}

		if (fabsf(state->magNormDip[0] - state->magRefNorm) < params->magNormTh * state->magRefNorm
		    && fabsf(state->magNormDip[1] - state->magRefDip) < params->magDipTh * (M_PIf / 180.0f)) {
			state->magUndisturbedT += Ts;
			if (state->magUndisturbedT >= params->magMinUndisturbedTime) {
				state->magDistDetected = false;
				state->magRefNorm += coeffs->kMagRef * (state->magNormDip[0] - state->magRefNorm);
				state->magRefDip  += coeffs->kMagRef * (state->magNormDip[1] - state->magRefDip);
			}
		} else {
			state->magUndisturbedT = 0.0f;
			state->magDistDetected = true;
		}

		if (fabsf(state->magNormDip[0] - state->magCandidateNorm) < params->magNormTh * state->magCandidateNorm
		    && fabsf(state->magNormDip[1] - state->magCandidateDip) < params->magDipTh * (M_PIf / 180.0f)) {
			if (norm(state->restLastGyrLp, 3) >= params->magNewMinGyr * M_PIf / 180.0f) {
				state->magCandidateT += Ts;
			}
			state->magCandidateNorm += coeffs->kMagRef * (state->magNormDip[0] - state->magCandidateNorm);
			state->magCandidateDip  += coeffs->kMagRef * (state->magNormDip[1] - state->magCandidateDip);
			if (state->magDistDetected && (state->magCandidateT >= params->magNewTime
			    || (state->magRefNorm <= 0.0f && state->magCandidateT >= params->magNewFirstTime))) {
				state->magRefNorm = state->magCandidateNorm;
				state->magRefDip  = state->magCandidateDip;
				state->magDistDetected = false;
				state->magUndisturbedT = params->magMinUndisturbedTime;
			}
		} else {
			state->magCandidateT = 0.0f;
			state->magCandidateNorm = state->magNormDip[0];
			state->magCandidateDip  = state->magNormDip[1];
		}
	}

	state->lastMagDisAngle = VQF_ATAN2(magEarth[0], magEarth[1]) - state->delta;
	if (state->lastMagDisAngle > M_PIf) { state->lastMagDisAngle -= 2 * M_PIf; }
	else if (state->lastMagDisAngle < -M_PIf) { state->lastMagDisAngle += 2 * M_PIf; }

	vqf_real_t k = coeffs->kMag;

	if (params->magDistRejectionEnabled) {
		if (state->magDistDetected) {
			if (state->magRejectT <= params->magMaxRejectionTime) {
				state->magRejectT += Ts; k = 0;
			} else { k /= params->magRejectionFactor; }
		} else {
			state->magRejectT = vqf_max(state->magRejectT - params->magRejectionFactor * Ts, 0.0f);
		}
	}

	if (state->kMagInit != 0.0f) {
		if (k < state->kMagInit) { k = state->kMagInit; }
		state->kMagInit = state->kMagInit / (state->kMagInit + 1);
		if (state->kMagInit * params->tauMag < Ts) { state->kMagInit = 0.0f; }
	}

	state->delta += k * state->lastMagDisAngle;
	state->lastMagCorrAngularRate = k * state->lastMagDisAngle / Ts;
	if (state->delta > M_PIf) { state->delta -= 2 * M_PIf; }
	else if (state->delta < -M_PIf) { state->delta += 2 * M_PIf; }
}

void updateMag(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs, const vqf_real_t mag[3])
{
	updateMag_internal(params, state, coeffs, coeffs->magTs, mag);
}

void updateMagTs(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
		 const vqf_real_t mag[3], uint64_t timestamp_us)
{
	vqf_real_t dt = coeffs->magTs;
	if (state->lastMagTsUs != 0 && timestamp_us > state->lastMagTsUs) {
		uint64_t diff = timestamp_us - state->lastMagTsUs;
		if (diff > 0 && diff <= 10000000ULL) { dt = (vqf_real_t)diff / 1e6f; }
	}
	updateMag_internal(params, state, coeffs, dt, mag);
	if (timestamp_us > state->lastMagTsUs || state->lastMagTsUs == 0) { state->lastMagTsUs = timestamp_us; }
}

/* ---------- getters ---------- */

void getQuat3D(vqf_state_t *const state, vqf_real_t out[4])
{
	memcpy(out, state->gyrQuat, sizeof(state->gyrQuat));
}

void getQuat6D(vqf_state_t *const state, vqf_real_t out[4])
{
	quatMultiply(state->accQuat, state->gyrQuat, out);
}

void getQuat9D(vqf_state_t *const state, vqf_real_t out[4])
{
	quatMultiply(state->accQuat, state->gyrQuat, out);
	quatApplyDelta(out, state->delta, out);
}

vqf_real_t getDelta(vqf_state_t *const state) { return state->delta; }

vqf_real_t getBiasEstimate(vqf_state_t *const state, vqf_coeffs_t *const coeffs, vqf_real_t out[3])
{
	if (out) { memcpy(out, state->bias, sizeof(state->bias)); }
	vqf_real_t sum1 = fabsf(state->biasP[0]) + fabsf(state->biasP[1]) + fabsf(state->biasP[2]);
	vqf_real_t sum2 = fabsf(state->biasP[3]) + fabsf(state->biasP[4]) + fabsf(state->biasP[5]);
	vqf_real_t sum3 = fabsf(state->biasP[6]) + fabsf(state->biasP[7]) + fabsf(state->biasP[8]);
	vqf_real_t P = vqf_min(vqf_max(vqf_max(sum1, sum2), sum3), coeffs->biasP0);
	return VQF_SQRT(P) * (M_PIf / 100.0f / 180.0f);
}

void setBiasEstimate(vqf_state_t *const state, vqf_real_t bias[3], vqf_real_t sigma)
{
	memcpy(state->bias, bias, sizeof(bias[0]) * 3);
	if (sigma > 0) {
		vqf_real_t P = vqf_square(sigma * (180.0f * 100.0f / M_PIf));
		matrix3SetToScaledIdentity(P, state->biasP);
	}
}

bool getRestDetected(vqf_state_t *const state)   { return state->restDetected; }
bool getMagDistDetected(vqf_state_t *const state) { return state->magDistDetected; }

void getRelativeRestDeviations(vqf_params_t *const params, vqf_state_t *const state, vqf_real_t out[2])
{
	out[0] = VQF_SQRT(state->restLastSquaredDeviations[0]) / (params->restThGyr * (M_PIf / 180.0f));
	out[1] = VQF_SQRT(state->restLastSquaredDeviations[1]) / params->restThAcc;
}

vqf_real_t getMagRefNorm(vqf_state_t *const state) { return state->magRefNorm; }
vqf_real_t getMagRefDip(vqf_state_t *const state)  { return state->magRefDip; }

void setMagRef(vqf_state_t *const state, vqf_real_t norm, vqf_real_t dip)
{
	state->magRefNorm = norm; state->magRefDip = dip;
}

/* ---------- setters ---------- */

void setMotionBiasEstEnabled(vqf_params_t *const params, vqf_state_t *const state, bool enabled)
{
	if (params->motionBiasEstEnabled == enabled) { return; }
	params->motionBiasEstEnabled = enabled;
	vqf_fill_double(state->motionBiasEstRLpState, 9*2, NaN);
	vqf_fill_double(state->motionBiasEstBiasLpState, 2*2, NaN);
}

void setRestBiasEstEnabled(vqf_params_t *const params, vqf_state_t *const state, bool enabled)
{
	if (params->restBiasEstEnabled == enabled) { return; }
	params->restBiasEstEnabled = enabled;
	state->restDetected = false;
	vqf_fill_real(state->restLastSquaredDeviations, 2, 0.0f);
	state->restT = 0.0f;
	vqf_fill_real(state->restLastGyrLp, 3, 0.0f);
	vqf_fill_double(state->restGyrLpState, 3*2, NaN);
	vqf_fill_real(state->restLastAccLp, 3, 0.0f);
	vqf_fill_double(state->restAccLpState, 3*2, NaN);
}

void setMagDistRejectionEnabled(vqf_params_t *const params, vqf_state_t *const state, bool enabled)
{
	if (params->magDistRejectionEnabled == enabled) { return; }
	params->magDistRejectionEnabled = enabled;
	state->magDistDetected = true;
	state->magRefNorm = 0.0f; state->magRefDip = 0.0f;
	state->magUndisturbedT = 0.0f;
	state->magRejectT = params->magMaxRejectionTime;
	state->magCandidateNorm = -1.0f; state->magCandidateDip = 0.0f; state->magCandidateT = 0.0f;
	vqf_fill_double(state->magNormDipLpState, 2*2, NaN);
}

void setTauAcc(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs, vqf_real_t tauAcc)
{
	if (params->tauAcc == tauAcc) { return; }
	params->tauAcc = tauAcc;
	vqf_double_t newB[3], newA[2];
	filterCoeffs(params->tauAcc, coeffs->accTs, newB, newA);
	filterAdaptStateForCoeffChange(state->lastAccLp, 3, coeffs->accLpB, coeffs->accLpA, newB, newA, state->accLpState);

	vqf_real_t R[9];
	for (size_t i = 0; i < 9; i++) { R[i] = state->motionBiasEstRLpState[2*i]; }
	filterAdaptStateForCoeffChange(R, 9, coeffs->accLpB, coeffs->accLpA, newB, newA, state->motionBiasEstRLpState);
	vqf_real_t biasLp[2];
	for (size_t i = 0; i < 2; i++) { biasLp[i] = state->motionBiasEstBiasLpState[2*i]; }
	filterAdaptStateForCoeffChange(biasLp, 2, coeffs->accLpB, coeffs->accLpA, newB, newA, state->motionBiasEstBiasLpState);

	memcpy(coeffs->accLpB, newB, sizeof(newB));
	memcpy(coeffs->accLpA, newA, sizeof(coeffs->accLpA));
}

void setTauMag(vqf_params_t *const params, vqf_coeffs_t *const coeffs, vqf_real_t tauMag)
{
	params->tauMag = tauMag;
	coeffs->kMag = gainFromTau(params->tauMag, coeffs->magTs);
}

void setRestDetectionThresholds(vqf_params_t *const params, vqf_real_t thGyr, vqf_real_t thAcc)
{
	params->restThGyr = thGyr; params->restThAcc = thAcc;
}

/* ---------- setup / reset ---------- */

static void setup(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs)
{
	filterCoeffs(params->tauAcc, coeffs->accTs, coeffs->accLpB, coeffs->accLpA);
	coeffs->kMag = gainFromTau(params->tauMag, coeffs->magTs);

	coeffs->biasP0 = vqf_square(params->biasSigmaInit * 100.0f);
	vqf_real_t safeBiasForgettingTime = vqf_max(params->biasForgettingTime, 1e-6f);
	coeffs->biasV = vqf_square(0.1f * 100.0f) * coeffs->accTs / safeBiasForgettingTime;

	vqf_real_t pMotion = vqf_square(params->biasSigmaMotion * 100.0f);
	coeffs->biasMotionW = vqf_square(pMotion) / coeffs->biasV + pMotion;
	coeffs->biasVerticalW = coeffs->biasMotionW / vqf_max(params->biasVerticalForgettingFactor, 1e-10f);

	vqf_real_t pRest = vqf_square(params->biasSigmaRest * 100.0f);
	coeffs->biasRestW = vqf_square(pRest) / coeffs->biasV + pRest;

	filterCoeffs(params->restFilterTau, coeffs->gyrTs, coeffs->restGyrLpB, coeffs->restGyrLpA);
	filterCoeffs(params->restFilterTau, coeffs->accTs, coeffs->restAccLpB, coeffs->restAccLpA);

	coeffs->kMagRef = gainFromTau(params->magRefTau, coeffs->magTs);
	if (params->magCurrentTau > 0) {
		filterCoeffs(params->magCurrentTau, coeffs->magTs, coeffs->magNormDipLpB, coeffs->magNormDipLpA);
	} else {
		vqf_fill_double(coeffs->magNormDipLpB, 3, NaN);
		vqf_fill_double(coeffs->magNormDipLpA, 2, NaN);
	}

	resetState(params, state, coeffs);
}

void resetState(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs)
{
	quatSetToIdentity(state->gyrQuat);
	quatSetToIdentity(state->accQuat);
	state->delta = 0.0f;

	state->restDetected = false;
	state->magDistDetected = true;

	vqf_fill_real(state->lastAccLp, 3, 0);
	vqf_fill_double(state->accLpState, 3*2, NaN);
	state->lastAccCorrAngularRate = 0.0f;

	state->kMagInit = 1.0f;
	state->lastMagDisAngle = 0.0f;
	state->lastMagCorrAngularRate = 0.0f;

	vqf_fill_real(state->bias, 3, 0);
	matrix3SetToScaledIdentity(coeffs->biasP0, state->biasP);

	vqf_fill_double(state->motionBiasEstRLpState, 9*2, NaN);
	vqf_fill_double(state->motionBiasEstBiasLpState, 2*2, NaN);

	vqf_fill_real(state->restLastSquaredDeviations, 2, 0.0f);
	state->restT = 0.0f;
	vqf_fill_real(state->restLastGyrLp, 3, 0.0f);
	vqf_fill_double(state->restGyrLpState, 3*2, NaN);
	vqf_fill_real(state->restLastAccLp, 3, 0.0f);
	vqf_fill_double(state->restAccLpState, 3*2, NaN);

	state->magRefNorm = 0.0f; state->magRefDip = 0.0f;
	state->magUndisturbedT = 0.0f;
	state->magRejectT = params->magMaxRejectionTime;
	state->magCandidateNorm = -1.0f; state->magCandidateDip = 0.0f; state->magCandidateT = 0.0f;
	vqf_fill_real(state->magNormDip, 2, 0.0f);
	vqf_fill_double(state->magNormDipLpState, 2*2, NaN);

	state->lastGyrTsUs = 0; state->lastAccTsUs = 0; state->lastMagTsUs = 0;
}

void initVqf(vqf_params_t *const params, vqf_state_t *const state, vqf_coeffs_t *const coeffs,
	     vqf_real_t gyrTs, vqf_real_t accTs, vqf_real_t magTs)
{
	vqf_real_t safeGyrTs = gyrTs > 0 ? gyrTs : 0.01f;
	coeffs->gyrTs = safeGyrTs;
	coeffs->accTs = accTs > 0 ? accTs : safeGyrTs;
	coeffs->magTs = magTs > 0 ? magTs : safeGyrTs;
	setup(params, state, coeffs);
}
