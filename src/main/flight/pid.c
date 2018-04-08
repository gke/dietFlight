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

#include <platform.h>

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/config_reset.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/sound_beeper.h"
#include "drivers/time.h"

#include "fc/fc_core.h"
#include "fc/fc_rc.h"

#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "io/gps.h"

#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#define USE_PAVEL // gke

FAST_RAM uint32_t targetPidLooptime;

const angle_index_t rcAliasToAngleIndexMap[] = { AI_ROLL, AI_PITCH };

static FAST_RAM float itermAccelerator = 1.0f;

static FAST_RAM filterApplyFnPtr dtermLpfApplyFn;
static FAST_RAM void *dtermFilterLpf[2];
static FAST_RAM filterApplyFnPtr ptermYawFilterApplyFn;
static FAST_RAM void *ptermYawFilter;
static FAST_RAM bool pidStabilisationEnabled;

FAST_RAM float RateP[3], RateI[3], RateD[3], RateOut[3];
static FAST_RAM float RateKp[3], RateKi[3], RateKd[3];
static FAST_RAM float maxVel[3];
static FAST_RAM float relaxFactor;
static FAST_RAM float dtermSetpointWeight;
static FAST_RAM float AngleKp, hrzGain, hrzTransition,
		hrzCutoffDegrees, hrzRatio;
static FAST_RAM float ITermWindupPointInv;
static FAST_RAM uint8_t hrzTiltExpertMode;
static FAST_RAM float itermLimit;

typedef union dtermFilterLpf_u {
	pt1Filter_t pt1Filter[2];
	biquadFilter_t biquadFilter[2];
	firFilterDenoise_t denoisingFilter[2];
} dtermFilterLpf_t;

static FAST_RAM float targetdT;
PG_REGISTER_WITH_RESET_TEMPLATE(pidConfig_t, pidConfig, PG_PID_CONFIG, 2)
;
#if defined(USE_GYRO_SPI_MPU6000) || defined(USE_GYRO_SPI_MPU6500)  || defined(USE_GYRO_SPI_ICM20689)
#define PID_PROCESS_DENOM_DEFAULT       4
#else
#define PID_PROCESS_DENOM_DEFAULT       2
#endif
#ifdef USE_RUNAWAY_TAKEOFF
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
		.pid_process_denom = PID_PROCESS_DENOM_DEFAULT,
		.runaway_takeoff_prevention = false,
		.runaway_takeoff_deactivate_throttle = 25, // throttle level % needed to accumulate deactivation time
		.runaway_takeoff_deactivate_delay = 500 // Accumulated time (in milliseconds) before deactivation in successful takeoff
);
#else
PG_RESET_TEMPLATE(pidConfig_t, pidConfig,
		.pid_process_denom = PID_PROCESS_DENOM_DEFAULT
);
#endif

PG_REGISTER_ARRAY_WITH_RESET_FN(pidProfile_t, MAX_PROFILE_COUNT, pidProfiles, PG_PID_PROFILE, 2)
;

void resetPidProfile(pidProfile_t *pidProfile) {
	RESET_CONFIG(pidProfile_t, pidProfile,
			.pid = {
				[PID_ROLL] = {40, 40, 20},
				[PID_PITCH] = {58, 50, 22},
				[PID_YAW] = {70, 45, 20},
				[PID_ALT] = {50, 0, 0},
				[PID_POS] = {15, 0, 0}, // POSHOLD_P * 100, POSHOLD_I * 100,
				[PID_POSR] = {34, 14, 53}, // POSHOLD_RATE_P * 10, POSHOLD_RATE_I * 100, POSHOLD_RATE_D * 1000,
				[PID_NAVR] = {25, 33, 83}, // NAV_P * 10, NAV_I * 100, NAV_D * 1000
				[PID_LEVEL] = {50, 50, 75},
				[PID_MAG] = {40, 0, 0},
				[PID_VEL] = {55, 55, 75}
			},

			.pidSumLimit = PIDSUM_LIMIT,
			.pidSumLimitYaw = PIDSUM_LIMIT_YAW,
			.yaw_lpf_hz = 0,
			// notch filters not used gke
			.dterm_lpf_hz = 100, // filtering ON by default
			.dterm_notch_hz = 260,
			.dterm_notch_cutoff = 160,
			.dterm_filter_type = FILTER_PT1,
			.dterm_filter_style = KD_FILTER_CLASSIC,
			.itermWindupPointPercent = 50,
			.vbatPidCompensation = 0,
			.pidAtMinThrottle = PID_STABILISATION_ON,
			.levelAngleLimit = 55,
			.setpointRelaxRatio = 100,
			.dtermSetpointWeight = 0,
			.yawRateAccelLimit = 100,
			.rateAccelLimit = 0,
			.itermThrottleThreshold = 350,
			.itermAcceleratorGain = 1000,
			// crash stuff not used
			.crash_time = 500, // ms
			.crash_delay = 0, // ms
			.crash_recovery_angle = 10, // degrees
			.crash_recovery_rate = 100, // degrees/second
			.crash_dthreshold = 50, // degrees/second/second
			.crash_gthreshold = 400, // degrees/second
			.crash_setpoint_threshold = 350, // degrees/second
			.crash_recovery = PID_CRASH_RECOVERY_OFF, // off by default
			//
			.horizon_tilt_effect = 75,
			.horizon_tilt_expert_mode = false,
			.crash_limit_yaw = 200,
			.itermLimit = 150
	);
} // resetPidProfile

void pgResetFn_pidProfiles(pidProfile_t *pidProfiles) {
	for (int i = 0; i < MAX_PROFILE_COUNT; i++)
		resetPidProfile(&pidProfiles[i]);
} // pgResetFn_pidProfiles

static void pidSetTargetLooptime(uint32_t pidLooptime) {
	targetPidLooptime = pidLooptime;
	targetdT = (float) targetPidLooptime * 0.000001f;
} // pidSetTargetLooptime

void pidResetITerm(void) {
	for (int a = 0; a < 3; a++)
		RateI[a] = 0.0f;
} // pidResetITerm

void pidSetItermAccelerator(float newItermAccelerator) {
	itermAccelerator = newItermAccelerator;
} // pidSetItermAccelerator

void pidStabilisationState(pidStabilisationState_e pidControllerState) {
	pidStabilisationEnabled
			= (pidControllerState == PID_STABILISATION_ON) ? true : false;
} // pidStabilisationState

// for derivative delta
struct {
	float b[10]; //6
} dFIRBuff[3];
static firFilter_t dFIR[3];

void pidInitFilters(const pidProfile_t *pidProfile) {
	BUILD_BUG_ON(FD_YAW != 2); // only setting up Dterm filters on roll and pitch axes, so ensure yaw axis is 2

	if (targetPidLooptime == 0) {
		dtermLpfApplyFn = nullFilterApply;
		ptermYawFilterApplyFn = nullFilterApply;
		return;
	}

	const float coeffsPavel[] = { 0.375f, 0.5f, -0.5f, -0.75, 0.125f, 0.25f };

	for (int a = FD_ROLL; a <= FD_YAW; a++) // inc yaw for later gke
#if defined(USE_PAVEL)
		firFilterInit(&dFIR[a], dFIRBuff[a].b, 6, coeffsPavel);
#else
	firFilterInit(&dFIR[a], dFIRBuff[a].b, 4, NULL);
#endif

	const uint32_t pidFrequencyNyquist = (1.0f / targetdT) / 2; // No rounding needed

	static dtermFilterLpf_t dtermFilterLpfUnion;
	if (pidProfile->dterm_lpf_hz == 0 || pidProfile->dterm_lpf_hz
			> pidFrequencyNyquist)
		dtermLpfApplyFn = nullFilterApply;
	else {
		dtermLpfApplyFn = (filterApplyFnPtr) pt1FilterApply;
		for (int a = FD_ROLL; a <= FD_PITCH; a++) {
			dtermFilterLpf[a] = &dtermFilterLpfUnion.pt1Filter[a];
			pt1FilterInit(dtermFilterLpf[a], pidProfile->dterm_lpf_hz, targetdT);
		}
	}

	static pt1Filter_t pt1FilterYaw;
	if (pidProfile->yaw_lpf_hz == 0 || pidProfile->yaw_lpf_hz
			> pidFrequencyNyquist)
		ptermYawFilterApplyFn = nullFilterApply;
	else {
		ptermYawFilterApplyFn = (filterApplyFnPtr) pt1FilterApply;
		ptermYawFilter = &pt1FilterYaw;
		pt1FilterInit(ptermYawFilter, pidProfile->yaw_lpf_hz, targetdT);
	}
} // pidInitFilters


void pidInitConfig(const pidProfile_t *pidProfile) {

	for (int a = FD_ROLL; a <= FD_YAW; a++) {
		RateKp[a] = PTERM_SCALE * pidProfile->pid[a].P;
		RateKi[a] = ITERM_SCALE * pidProfile->pid[a].I;
		RateKd[a] = DTERM_SCALE * pidProfile->pid[a].D;
	}
	dtermSetpointWeight = pidProfile->dtermSetpointWeight * 0.01f; // %
	relaxFactor = 1.0f / (pidProfile->setpointRelaxRatio * 0.01f);
	AngleKp = pidProfile->pid[PID_LEVEL].P * 0.1f;
	hrzGain = pidProfile->pid[PID_LEVEL].I * 0.1f;
	hrzTransition = (float) pidProfile->pid[PID_LEVEL].D;
	hrzTiltExpertMode = pidProfile->horizon_tilt_expert_mode;
	hrzCutoffDegrees = (175.0f - pidProfile->horizon_tilt_effect) * 1.8f;
	hrzRatio = (100.0f - pidProfile->horizon_tilt_effect) * 0.01f;
	maxVel[FD_ROLL] = maxVel[FD_PITCH] = pidProfile->rateAccelLimit * 100.0f
			* targetdT;
	maxVel[FD_YAW] = pidProfile->yawRateAccelLimit * 100.0f * targetdT;
	const float ITermWindupPoint = (float) pidProfile->itermWindupPointPercent
			* 0.01f;
	ITermWindupPointInv = 1.0f / (1.0f - ITermWindupPoint);
	itermLimit = pidProfile->itermLimit;
} // pidInitConfig


void pidInit(const pidProfile_t *pidProfile) {
	pidSetTargetLooptime(gyro.targetLooptime * pidConfig()->pid_process_denom);
	pidInitFilters(pidProfile);
	pidInitConfig(pidProfile);
} // pidInit

void pidCopyProfile(uint8_t dstPidProfileIndex, uint8_t srcPidProfileIndex) {
	if ((dstPidProfileIndex < MAX_PROFILE_COUNT - 1 && srcPidProfileIndex
			< MAX_PROFILE_COUNT - 1) && dstPidProfileIndex
			!= srcPidProfileIndex)
		memcpy(pidProfilesMutable(dstPidProfileIndex), pidProfilesMutable(
				srcPidProfileIndex), sizeof(pidProfile_t));
} // pidCopyProfile

static float calcHorizonLevelStrength(void) {
	float currInclination, levelStrength, sensitFact,
			tiltRatio;

	levelStrength = 1.0f - MAX(getRcDeflectionAbs(FD_ROLL),
			getRcDeflectionAbs(FD_PITCH));

	currInclination = MAX(ABS(attitude.values.roll), ABS(
					attitude.values.pitch)) * 0.1f;

	if (hrzRatio < 1.0f) { // if hrzTiltEffect > 0
		tiltRatio = (180.0f - currInclination) / 180.0f * (1.0f
				- hrzRatio) + hrzRatio;
		sensitFact = hrzTransition * tiltRatio;
	} else
		sensitFact = hrzTransition;

	levelStrength = (sensitFact <= 0.0f) ? 0.0f : ((levelStrength
			- 1.0f) * (100.0f / sensitFact)) + 1.0f;

	return constrainf(levelStrength, 0.0f, 1.0f);
} // calcHorizonLevelStrength

static float pidLevel(int a, const pidProfile_t *pidProfile,
		const rollAndPitchTrims_t * angleTrim, float desiredRate) {
	float levelStrength, Angle, AngleE;

	Angle = pidProfile->levelAngleLimit * getRcDeflection(a); // Rc is +/- 1
#ifdef USE_GPS
	Angle += GPS_angle[a];
#endif
	Angle = constrain1f(Angle, pidProfile->levelAngleLimit);
	AngleE = Angle - ((attitude.raw[a] - angleTrim->raw[a]) * 0.1f);

	if (FLIGHT_MODE(ANGLE_MODE))
		desiredRate = AngleE * AngleKp;
	else {
		levelStrength = calcHorizonLevelStrength();
		desiredRate = desiredRate + (AngleE * hrzGain
				* levelStrength);
	}
	return desiredRate;
} // pidLevel


static float limitAcc(int a, float desiredRate) {
	static float prevdesiredRate[3] = { 0.0f, };
	float currVel;

	currVel = desiredRate - prevdesiredRate[a];

	if (ABS(currVel) > maxVel[a])
		desiredRate = (currVel > 0.0f) ? prevdesiredRate[a] + maxVel[a]
				: prevdesiredRate[a] - maxVel[a];

	prevdesiredRate[a] = desiredRate;
	return desiredRate;
} // limitAcc


float rateDerivative(int a, float RateE, float dTR) {
	// UAVX using "rate on measurement" to avoid "derivative kick"

#if defined(USE_PAVEL)
	return dtermLpfApplyFn(dtermFilterLpf[a], firFilterUpdateAndApply(&dFIR[a],
			RateE) * dTR);
#else
	float RateDelta, r;
	static float prevRateE[3] = {0.0f,}; // inc yaw for now

	r = dtermLpfApplyFn(dtermFilterLpf[a], RateE); // should be a lower frequency than gyro gke
	RateDelta = (r - prevRateE[a]) * dTR;
	prevRateE[a] = r;

	firFilterUpdateAverage(dFIR[a], RateDelta);

	return dFIR[a]->movingSum * 0.25f;
#endif

} // rateDerivative


void pidController(const pidProfile_t *pidProfile,
		const rollAndPitchTrims_t *angleTrim, timeUs_t currTimeUs) {
	float motorMixRange, dT, dTR, Rate, RateE, desiredRate, dynCi, ITerm,
			ITermNew, tpaFactor;
	static timeUs_t prevTimeUs;

	tpaFactor = getThrottlePIDAttenuation();
	motorMixRange = getMotorMixRange();

	dT = (currTimeUs - prevTimeUs) * 0.000001f;
	dTR = 1.0f / dT;
	prevTimeUs = currTimeUs;

	// Dynamic i component,
	// gradually scale back integration when above windup point,
	// use target dT (not actual dT) for ITerm calculation to avoid wind-up caused by jitter
	dynCi = MIN((1.0f - motorMixRange) * ITermWindupPointInv, 1.0f) * dT
			* itermAccelerator; // was targetdT

	for (int a = FD_ROLL; a <= FD_YAW; a++) {
		desiredRate = getSetpointRate(a);
		if (maxVel[a] != 0.0f)
			desiredRate = limitAcc(a, desiredRate);

		if ((FLIGHT_MODE(ANGLE_MODE) || FLIGHT_MODE(HORIZON_MODE)) && a != YAW)
			desiredRate = pidLevel(a, pidProfile, angleTrim, desiredRate);

		Rate = gyro.gyroADCf[a];
		RateE = desiredRate - Rate;

		RateP[a] = RateE * RateKp[a] * tpaFactor;

		ITerm = RateI[a];
		ITermNew = constrain1f(ITerm + RateE * RateKi[a] * dynCi, itermLimit);
		if ((ABS(ITermNew) < ABS(ITerm)) || !mixerIsOutputSaturated(a, RateE)) // windup limiting
			RateI[a] = ITermNew;

		if (a == FD_YAW) {
			RateP[a] = ptermYawFilterApplyFn(ptermYawFilter, RateP[a]);
			RateOut[a] = RateP[a] + RateI[a];
		} else {
			RateD[a] = RateKd[a] * rateDerivative(a, RateE, dTR) * tpaFactor;
			RateOut[a] = RateP[a] + RateI[a] + RateD[a];
		}

		if (!pidStabilisationEnabled)
			RateP[a] = RateI[a] = RateD[a] = RateOut[a] = 0.0f;
	}
} // pidController


