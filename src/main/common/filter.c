/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#include "common/filter.h"
#include "common/maths.h"
#include "common/utils.h"

#define M_LN2_FLOAT 0.69314718055994530942f
#define M_PI_FLOAT  3.14159265358979323846f
#define BIQUAD_BANDWIDTH 1.9f     /* bandwidth in octaves */
#define BIQUAD_Q 1.0f / sqrtf(2.0f)     /* quality factor - butterworth*/

// NULL filter

FAST_CODE float nullFilterApply(filter_t *filter, float input) {
	UNUSED(filter);
	return input;
}

// PT1 Low Pass filter

void pt1FilterInit(pt1Filter_t *filter, uint16_t f_cut, float dT) {
	float RC = 1.0f / (2.0f * M_PI_FLOAT * (float)f_cut);
	filter->k = dT / (RC + dT);
}

FAST_CODE float pt1FilterApply(pt1Filter_t *filter, float input) {
	filter->state = filter->state + filter->k * (input - filter->state);
	return filter->state;
} // pt1FilterApply

// PTn Low Pass filter

void ptnFilterInit(ptnFilter_t *filter, uint8_t order, uint16_t f_cut, float dT) {

	// AdjCutHz = CutHz /(sqrtf(powf(2, 1/Order) -1))
	const float ScaleF[] = { 1.0f, 1.553773974f, 1.961459177f, 2.298959223f };
	int n;
	float Adj_f_cut;

	filter->order = (order > 4) ? 4 : order;
	for (n = 1; n <= filter->order; n++)
		filter->state[n] = 0.0f;

	Adj_f_cut = (float)f_cut * ScaleF[filter->order - 1];

	filter->k = dT / ((1.0f / (2.0f * M_PI_FLOAT * Adj_f_cut)) + dT);

} // ptnFilterInit

FAST_CODE float ptnFilterApply(ptnFilter_t *filter, float input) {
int n;

	filter->state[0] = input;

	for (n = 1; n <= filter->order; n++)
		filter->state[n] += (filter->state[n - 1] - filter->state[n])
				* filter->k;

	return filter->state[filter->order];
} // ptnFilterApply


// Slew filter with limit

void slewFilterInit(slewFilter_t *filter, float slewLimit, float threshold) {
	filter->state = 0.0f;
	filter->slewLimit = slewLimit;
	filter->threshold = threshold;
} // slewFilterInit

FAST_CODE float slewFilterApply(slewFilter_t *filter, float input) {
	if (filter->state >= filter->threshold) {
		if (input >= filter->state - filter->slewLimit) {
			filter->state = input;
		}
	} else if (filter->state <= -filter->threshold) {
		if (input <= filter->state + filter->slewLimit) {
			filter->state = input;
		}
	} else {
		filter->state = input;
	}
	return filter->state;
}

/* sets up a biquad Filter */
void biquadFilterInitLPF(biquadFilter_t *filter, float filterFreq,
		uint32_t refreshRate) {
	biquadFilterInit(filter, filterFreq, refreshRate, BIQUAD_Q, FILTER_LPF);
}

void biquadFilterInit(biquadFilter_t *filter, float filterFreq,
		uint32_t refreshRate, float Q, biquadFilterType_e filterType) {
	// setup variables
	const float omega = 2.0f * M_PI_FLOAT * filterFreq * refreshRate
			* 0.000001f;
	const float sn = sin_approx(omega);
	const float cs = cos_approx(omega);
	const float alpha = sn / (2.0f * Q);

	float b0 = 0, b1 = 0, b2 = 0, a0 = 0, a1 = 0, a2 = 0;

	switch (filterType) {
	case FILTER_LPF:
		b0 = (1 - cs) * 0.5f;
		b1 = 1 - cs;
		b2 = (1 - cs) * 0.5f;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	case FILTER_NOTCH:
		b0 = 1;
		b1 = -2 * cs;
		b2 = 1;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	case FILTER_BPF:
		b0 = alpha;
		b1 = 0;
		b2 = -alpha;
		a0 = 1 + alpha;
		a1 = -2 * cs;
		a2 = 1 - alpha;
		break;
	}

	// precompute the coefficients
	filter->b0 = b0 / a0;
	filter->b1 = b1 / a0;
	filter->b2 = b2 / a0;
	filter->a1 = a1 / a0;
	filter->a2 = a2 / a0;

	// zero initial samples
	filter->x1 = filter->x2 = 0;
	filter->y1 = filter->y2 = 0;
}

FAST_CODE void biquadFilterUpdate(biquadFilter_t *filter, float filterFreq,
		uint32_t refreshRate, float Q, biquadFilterType_e filterType) {
	// backup state
	float x1 = filter->x1;
	float x2 = filter->x2;
	float y1 = filter->y1;
	float y2 = filter->y2;

	biquadFilterInit(filter, filterFreq, refreshRate, Q, filterType);

	// restore state
	filter->x1 = x1;
	filter->x2 = x2;
	filter->y1 = y1;
	filter->y2 = y2;
}

/* Computes a biquadFilter_t filter on a sample (slightly less precise than df2 but works in dynamic mode) */
FAST_CODE float biquadFilterApplyDF1(biquadFilter_t *filter, float input) {
	/* compute result */
	const float result = filter->b0 * input + filter->b1 * filter->x1
			+ filter->b2 * filter->x2 - filter->a1 * filter->y1 - filter->a2
			* filter->y2;

	/* shift x1 to x2, input to x1 */
	filter->x2 = filter->x1;
	filter->x1 = input;

	/* shift y1 to y2, result to y1 */
	filter->y2 = filter->y1;
	filter->y1 = result;

	return result;
}

/* Computes a biquadFilter_t filter in direct form 2 on a sample (higher precision but can't handle changes in coefficients */
FAST_CODE float biquadFilterApply(biquadFilter_t *filter, float input) {
	const float result = filter->b0 * input + filter->x1;
	filter->x1 = filter->b1 * input - filter->a1 * result + filter->x2;
	filter->x2 = filter->b2 * input - filter->a2 * result;
	return result;
}

/*
 * FIR filter
 */
void firFilterInit2(firFilter_t *filter, float *buf, uint8_t bufLength,
		const float *coeffs, uint8_t coeffsLength) {
	filter->buf = buf;
	filter->bufLength = bufLength;
	filter->coeffs = coeffs;
	filter->coeffsLength = coeffsLength;
	filter->movingSum = 0.0f;
	filter->index = 0;
	filter->count = 0;
	memset(filter->buf, 0, sizeof(float) * filter->bufLength);
}

/*
 * FIR filter initialisation
 * If the FIR filter is just to be used for averaging, then coeffs can be set to NULL
 */
void firFilterInit(firFilter_t *filter, float *buf, uint8_t bufLength,
		const float *coeffs) {
	firFilterInit2(filter, buf, bufLength, coeffs, bufLength);
}

void firFilterUpdate(firFilter_t *filter, float input) {
	filter->buf[filter->index++] = input; // index is at the first empty buffer positon
	if (filter->index >= filter->bufLength) {
		filter->index = 0;
	}
}

/*
 * Update FIR filter maintaining a moving sum for quick moving average computation
 */
void firFilterUpdateAverage(firFilter_t *filter, float input) {
	filter->movingSum += input; // sum of the last <count> items, to allow quick moving average computation
	filter->movingSum -= filter->buf[filter->index]; // subtract the value that "drops off" the end of the moving sum
	filter->buf[filter->index++] = input; // index is at the first empty buffer positon
	if (filter->index >= filter->bufLength) {
		filter->index = 0;
	}
	if (filter->count < filter->bufLength) {
		++filter->count;
	}
}

FAST_CODE float firFilterApply(const firFilter_t *filter) {
	float ret = 0.0f;
	int ii = 0;
	int index;
	for (index = filter->index - 1; index >= 0; ++ii, --index) {
		ret += filter->coeffs[ii] * filter->buf[index];
	}
	for (index = filter->bufLength - 1; ii < filter->coeffsLength; ++ii, --index) {
		ret += filter->coeffs[ii] * filter->buf[index];
	}
	return ret;
}

FAST_CODE float firFilterUpdateAndApply(firFilter_t *filter, float input) {
	firFilterUpdate(filter, input);
	return firFilterApply(filter);
}

/*
 * Returns average of the last <count> items.
 */
float firFilterCalcPartialAverage(const firFilter_t *filter, uint8_t count) {
	float ret = 0.0f;
	int index = filter->index;
	for (int ii = 0; ii < filter->coeffsLength; ++ii) {
		--index;
		if (index < 0) {
			index = filter->bufLength - 1;
		}
		ret += filter->buf[index];
	}
	return ret / count;
}

float firFilterCalcMovingAverage(const firFilter_t *filter) {
	return filter->movingSum / filter->count;
}

float firFilterLastInput(const firFilter_t *filter) {
	// filter->index points to next empty item in buffer
	const int index = filter->index == 0 ? filter->bufLength - 1
			: filter->index - 1;
	return filter->buf[index];
}

void firFilterDenoiseInit(firFilterDenoise_t *filter, uint8_t gyroSoftLpfHz,
		uint16_t targetLooptime) {
	memset(filter, 0, sizeof(firFilterDenoise_t));
	filter->targetCount = constrain(lrintf((1.0f / (0.000001f
			* (float) targetLooptime)) / gyroSoftLpfHz), 1,
			MAX_FIR_DENOISE_WINDOW_SIZE);
} // firFilterDenoiseInit

// prototype function for de-noising of signal by dynamic moving average. Mainly for test purposes
float firFilterDenoiseUpdate(firFilterDenoise_t *filter, float input) {
	filter->state[filter->index] = input;
	filter->movingSum += filter->state[filter->index++];
	if (filter->index == filter->targetCount)
		filter->index = 0;

	filter->movingSum -= filter->state[filter->index];

	if (filter->targetCount >= filter->filledCount)
		return filter->movingSum / filter->targetCount;
	else
		return filter->movingSum / ++filter->filledCount + 1;

} // firFilterDenoiseUpdate


// rs2k's fast "kalman" filter per Fujin
void fixedKKalmanInit(fastKalman_t *filter, uint16_t f_cut, float dT) {
	float RC = 1.0f / (2.0f * M_PI_FLOAT * f_cut);
	float a = dT / (RC + dT);

	filter->x = 0.0f; // set initial value, can be zero if unknown
	filter->lastX = 0.0f; // set initial value, can be zero if unknown
	filter->k = a * 0.5f; // "kalman" gain - half of RC coefficient
}

FAST_CODE float fixedKKalmanUpdate(fastKalman_t *filter, float input) {
	filter->x += (filter->x - filter->lastX);
	filter->lastX = filter->x;
	filter->x += filter->k * (input - filter->x);
	return filter->x;
} //fixedKKalmanInit


// Fast two-state Kalman
void fastKalmanInit(fastKalman_t *filter, float q, float r, float p) {
	filter->q = q * 0.000001f; // add multiplier to make tuning easier
	filter->r = r * 0.001f; // add multiplier to make tuning easier
	filter->p = p * 0.001f; // add multiplier to make tuning easier
	filter->x = 0.0f; // set initial value, can be zero if unknown
	filter->lastX = 0.0f; // set initial value, can be zero if unknown
	filter->k = 0.0f; // kalman gain
} // fastKalmanInit

FAST_CODE float fastKalmanUpdate(fastKalman_t *filter, float input) {
	// project the state ahead using acceleration
	filter->x += (filter->x - filter->lastX);

	// update last state
	filter->lastX = filter->x;

	// prediction update
	filter->p = filter->p + filter->q;

	// measurement update
	filter->k = filter->p / (filter->p + filter->r);
	filter->x += filter->k * (input - filter->x);
	filter->p = (1.0f - filter->k) * filter->p;

	return filter->x;
} // fastKalmanUpdate


// Robert Bouwens AlphaBetaGamma

void initABGCoeffs(ABG_t* pab, float x_measured, float alpha, float beta) {
	pab->xk_1 = x_measured;
	pab->vk_1 = 0;
	pab->a = alpha;
	pab->b = beta;
	pab->g = 0.0f;
} // InitializeAlphaBeta

// near critically damped filter
void createNearCriticalFilter(ABG_t* pab, float x_measured, float alpha){
	const float beta = 0.8f * (2.0f - alpha * alpha - 2.0f * sqrtf(1.0f - alpha * alpha)) / (alpha * alpha);

	initABGCoeffs(pab, x_measured, alpha, beta);
	pab->g = pab->b * pab->b / (pab->a * 2.0f);
} // createNearCriticalFilter

void createUnderDampedFilter(ABG_t* pab, float x_measured, float alpha) {
	const float beta = alpha * alpha / (2.0f - alpha); /*  standard, underdamped beta value */

	initABGCoeffs(pab, x_measured, alpha, beta);
	pab->g = pab->b * pab->b / (pab->a * 2.0f);
} // createUnderDampedFilter


void ABGInit(ABG_t *filter, float q, float r, float p, float dT) {
	(void) p;
	const float Q = q * 0.001f;
	const float R = r * 0.001f;
	//    const float ALPHA = 0.85;

	 createNearCriticalFilter(filter, 0.0f, Q);
	//    calculateAlphaBetaGamma(filter, Q, R, dT);
	//    createUnderDampedFilter(filter, 0.0f,  ALPHA);
	//    peter_nachtwey(filter);

	filter->dT = dT;
	filter->dT2 = dT * dT;
	filter->xk_1 = 0.0f;
} // ABGInit


FAST_CODE float ABGUpdate(ABG_t *pab, float input) {
	float xk_1 = pab->xk_1;
	float vk_1 = pab->vk_1;
	float ak_1 = pab->ak_1;
	float alpha = pab->a;
	float beta = pab->b;
	float gamma = pab->g;

	//    float xk;   // current system state (ie: position)
	//    float vk;   // derivative of system state (ie: velocity)
	float rk; // residual error

	// update our (estimated) state 'x' from the system (ie pos = pos + vel (last).dT)
	xk_1 += pab->dT * vk_1 + 0.5f * pab->dT2 * ak_1;
	// update (estimated) velocity
    vk_1 += pab->dT * ak_1;
	// what is our residual error (measured - estimated)
	rk = input - xk_1;
	// update our estimates given the residual error.
	xk_1 += alpha * rk;
	vk_1 += beta / pab->dT * rk;
	if (gamma != 0.0f)
		ak_1 = ak_1 + gamma / (2.0f * pab->dT2) * rk;

	pab->vk_1 = vk_1;
	pab->xk_1 = xk_1;
	pab->ak_1 = ak_1;

	return xk_1;
} // ABGUpdate

