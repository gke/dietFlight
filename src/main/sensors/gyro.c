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
#include <stdlib.h>

#include "platform.h"

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/feature.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/accgyro/accgyro_adxl345.h"
#include "drivers/accgyro/accgyro_bma280.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/accgyro/accgyro_l3g4200d.h"
#include "drivers/accgyro/accgyro_l3gd20.h"
#include "drivers/accgyro/accgyro_lsm303dlhc.h"
#include "drivers/accgyro/accgyro_mma845x.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/accgyro/accgyro_mpu3050.h"
#include "drivers/accgyro/accgyro_mpu6050.h"
#include "drivers/accgyro/accgyro_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_bmi160.h"
#include "drivers/accgyro/accgyro_spi_icm20649.h"
#include "drivers/accgyro/accgyro_spi_icm20689.h"
#include "drivers/accgyro/accgyro_spi_mpu6000.h"
#include "drivers/accgyro/accgyro_spi_mpu6500.h"
#include "drivers/accgyro/accgyro_spi_mpu9250.h"
#ifdef USE_GYRO_IMUF9001
#include "drivers/accgyro/accgyro_imuf9001.h"
#include "drivers/time.h"
#endif //USE_GYRO_IMUF9001
#include "drivers/accgyro/gyro_sync.h"
#include "drivers/bus_spi.h"
#include "drivers/io.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/runtime_config.h"

#include "io/beeper.h"
#include "io/statusindicator.h"

#include "scheduler/scheduler.h"

#include "sensors/boardalignment.h"
#include "sensors/gyro.h"
#include "sensors/gyroanalyse.h"
#include "sensors/sensors.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

FAST_RAM gyro_t gyro;
static FAST_RAM uint8_t gyroDebugMode;

static FAST_RAM float accumulatedMeasurements[XYZ_AXIS_COUNT];
static FAST_RAM float previousgyroADCf[XYZ_AXIS_COUNT];
static FAST_RAM timeUs_t accumulatedMeasurementTimeUs;
static FAST_RAM timeUs_t accumulationLastTimeSampledUs;

typedef struct gyroCalibration_s {
	int32_t sum[XYZ_AXIS_COUNT];
	stdev_t var[XYZ_AXIS_COUNT];
	uint16_t calibratingG;
} gyroCalibration_t;

bool firstArmingCalibrationWasStarted = false;

typedef union gyroSoftFilter_u {
	biquadFilter_t gyroFilterLpfState[XYZ_AXIS_COUNT];
	pt1Filter_t gyroFilterPt1State[XYZ_AXIS_COUNT];
	ptnFilter_t gyroFilterPtnState[XYZ_AXIS_COUNT];
	firFilterDenoise_t gyroDenoiseState[XYZ_AXIS_COUNT];
} gyroSoftLpfFilter_t;

typedef struct gyroSensor_s {
	gyroDev_t gyroDev;
	gyroCalibration_t calibration;
	// gyro soft filter
	filterApplyFnPtr softLpfFilterApplyFn;
	gyroSoftLpfFilter_t softLpfFilter;
	filter_t *softLpfFilterPtr[XYZ_AXIS_COUNT];
	bool overflowDetected[XYZ_AXIS_COUNT];
	// gyro kalman filter
	filterApplyFnPtr fastKalmanApplyFn;
	filterApplyFnPtr fixedKKalmanApplyFn;
	fastKalman_t fastKalman[XYZ_AXIS_COUNT];

} gyroSensor_t;
STATIC_UNIT_TESTED FAST_RAM gyroSensor_t gyroSensor1;
#ifdef UNIT_TEST
STATIC_UNIT_TESTED gyroSensor_t * const gyroSensorPtr = &gyroSensor1;
STATIC_UNIT_TESTED gyroDev_t * const gyroDevPtr = &gyroSensor1.gyroDev;
#endif

static void gyroInitFilterKalman(gyroSensor_t *gyroSensor,
		uint16_t gyro_filter_q, uint16_t gyro_filter_r, uint16_t gyro_filter_p);
static void
gyroInitFilterFixedKKalman(gyroSensor_t *gyroSensor, uint16_t lpfHz);

static void gyroInitSensorFilters(gyroSensor_t *gyroSensor);
#define DEBUG_GYRO_CALIBRATION 3
PG_REGISTER_WITH_RESET_TEMPLATE(gyroConfig_t, gyroConfig, PG_GYRO_CONFIG, 1)
;
PG_RESET_TEMPLATE(gyroConfig_t, gyroConfig,
		.gyro_align = ALIGN_DEFAULT,
		.gyroMovementCalibrationThreshold = 48,
		.gyro_sync_denom = 1, // GYRO_SYNC_DENOM_DEFAULT,
		.gyro_lpf = GYRO_LPF_NONE,
		.gyro_soft_lpf_type = FILTER_PT1, // should PTn gke
		.gyro_soft_lpf_hz = 100, // gke
		.gyro_high_fsr = false,
		.gyro_use_32khz = false,
		.gyro_to_use = 0,
		.gyro_soft_notch_hz_1 = 0, // not used gke
		.gyro_soft_notch_cutoff_1 = 0, // not used gke
		.gyro_soft_notch_hz_2 = 0, // not used gke
		.gyro_soft_notch_cutoff_2 = 0, // not used gke
		.checkOverflow = GYRO_OVERFLOW_CHECK_YAW,
		.gyro_soft_lpf_hz_2 = 0, // Fujin/FIXED KF and PTn forced to Nyquist if zero gke
		.gyro_filter_q = 2000, // was 800 should be around 2000 for 8KHz for Kalyn KF  gke
		.gyro_filter_r = 88,
		.gyro_filter_p = 0,
		.gyro_stage2_filter_type = STAGE2_FILTER_FIXED_K_KALMAN,
		.gyro_offset_yaw = 0,
);

const busDevice_t *gyroSensorBus(void) {
	return &gyroSensor1.gyroDev.bus;
} // gyroSensorBus

const mpuConfiguration_t *gyroMpuConfiguration(void) {
	return &gyroSensor1.gyroDev.mpuConfiguration;
} // gyroMpuConfiguration

const mpuDetectionResult_t *gyroMpuDetectionResult(void) {
	return &gyroSensor1.gyroDev.mpuDetectionResult;
} // gyroMpuDetectionResult

STATIC_UNIT_TESTED gyroSensor_e gyroDetect(gyroDev_t *dev) {
	gyroSensor_e gyroHardware = GYRO_DEFAULT;

	dev->gyroAlign = ALIGN_DEFAULT;

	switch (gyroHardware) {
	case GYRO_DEFAULT:
		FALLTHROUGH;

#ifdef USE_GYRO_MPU6050
		case GYRO_MPU6050:
		if (mpu6050GyroDetect(dev)) {
			gyroHardware = GYRO_MPU6050;
#ifdef GYRO_MPU6050_ALIGN
			dev->gyroAlign = GYRO_MPU6050_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif

#ifdef USE_GYRO_L3G4200D
		case GYRO_L3G4200D:
		if (l3g4200dDetect(dev)) {
			gyroHardware = GYRO_L3G4200D;
#ifdef GYRO_L3G4200D_ALIGN
			dev->gyroAlign = GYRO_L3G4200D_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif

#ifdef USE_GYRO_MPU3050
		case GYRO_MPU3050:
		if (mpu3050Detect(dev)) {
			gyroHardware = GYRO_MPU3050;
#ifdef GYRO_MPU3050_ALIGN
			dev->gyroAlign = GYRO_MPU3050_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif

#ifdef USE_GYRO_L3GD20
		case GYRO_L3GD20:
		if (l3gd20Detect(dev)) {
			gyroHardware = GYRO_L3GD20;
#ifdef GYRO_L3GD20_ALIGN
			dev->gyroAlign = GYRO_L3GD20_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_MPU6000
	case GYRO_MPU6000:
		if (mpu6000SpiGyroDetect(dev)) {
			gyroHardware = GYRO_MPU6000;
#ifdef GYRO_MPU6000_ALIGN
			dev->gyroAlign = GYRO_MPU6000_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif

#if defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500)
	case GYRO_MPU6500:
	case GYRO_ICM20601:
	case GYRO_ICM20602:
	case GYRO_ICM20608G:
#ifdef USE_GYRO_SPI_MPU6500
		if (mpu6500GyroDetect(dev) || mpu6500SpiGyroDetect(dev)) {
#else
			if (mpu6500GyroDetect(dev)) {
#endif
			switch (dev->mpuDetectionResult.sensor) {
			case MPU_9250_SPI:
				gyroHardware = GYRO_MPU9250;
				break;
			case ICM_20601_SPI:
				gyroHardware = GYRO_ICM20601;
				break;
			case ICM_20602_SPI:
				gyroHardware = GYRO_ICM20602;
				break;
			case ICM_20608_SPI:
				gyroHardware = GYRO_ICM20608G;
				break;
			default:
				gyroHardware = GYRO_MPU6500;
			}
#ifdef GYRO_MPU6500_ALIGN
			dev->gyroAlign = GYRO_MPU6500_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_MPU9250
		case GYRO_MPU9250:
		if (mpu9250SpiGyroDetect(dev)) {
			gyroHardware = GYRO_MPU9250;
#ifdef GYRO_MPU9250_ALIGN
			dev->gyroAlign = GYRO_MPU9250_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_ICM20649
		case GYRO_ICM20649:
		if (icm20649SpiGyroDetect(dev)) {
			gyroHardware = GYRO_ICM20649;
#ifdef GYRO_ICM20649_ALIGN
			dev->gyroAlign = GYRO_ICM20649_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif

#ifdef USE_GYRO_SPI_ICM20689
		case GYRO_ICM20689:
		if (icm20689SpiGyroDetect(dev)) {
			gyroHardware = GYRO_ICM20689;
#ifdef GYRO_ICM20689_ALIGN
			dev->gyroAlign = GYRO_ICM20689_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif

#ifdef USE_ACCGYRO_BMI160
		case GYRO_BMI160:
		if (bmi160SpiGyroDetect(dev)) {
			gyroHardware = GYRO_BMI160;
#ifdef GYRO_BMI160_ALIGN
			dev->gyroAlign = GYRO_BMI160_ALIGN;
#endif
			break;
		}
		FALLTHROUGH;
#endif
#ifdef USE_FAKE_GYRO
		case GYRO_FAKE:
		if (fakeGyroDetect(dev)) {
			gyroHardware = GYRO_FAKE;
			break;
		}
		FALLTHROUGH;
#endif
	default:
		gyroHardware = GYRO_NONE;
	}

	if (gyroHardware != GYRO_NONE) {
		detectedSensors[SENSOR_INDEX_GYRO] = gyroHardware;
		sensorsSet(SENSOR_GYRO);
	}

	return gyroHardware;
} // gyroDetect

static bool gyroInitSensor(gyroSensor_t *gyroSensor) {
#if defined(USE_GYRO_MPU6050) || defined(USE_GYRO_MPU3050) || defined(USE_GYRO_MPU6500) || defined(USE_GYRO_SPI_MPU6500) || defined(USE_GYRO_SPI_MPU6000) \
 || defined(USE_ACC_MPU6050) || defined(USE_GYRO_SPI_MPU9250) || defined(USE_GYRO_SPI_ICM20601) || defined(USE_GYRO_SPI_ICM20649) || defined(USE_GYRO_SPI_ICM20689)

#if defined(MPU_INT_EXTI)
	gyroSensor->gyroDev.mpuIntExtiTag = IO_TAG(MPU_INT_EXTI);
#elif defined(USE_HARDWARE_REVISION_DETECTION)
	gyroSensor->gyroDev.mpuIntExtiTag = selectMPUIntExtiConfigByHardwareRevision();
#else
	gyroSensor->gyroDev.mpuIntExtiTag = IO_TAG_NONE;
#endif // MPU_INT_EXTI
#ifdef USE_DUAL_GYRO
	// set cnsPin using GYRO_n_CS_PIN defined in target.h
	gyroSensor->gyroDev.bus.busdev_u.spi.csnPin = gyroConfig()->gyro_to_use == 0 ? IOGetByTag(IO_TAG(GYRO_0_CS_PIN)) : IOGetByTag(IO_TAG(GYRO_1_CS_PIN));
#else
	gyroSensor->gyroDev.bus.busdev_u.spi.csnPin = IO_NONE; // set cnsPin to IO_NONE so mpuDetect will set it according to value defined in target.h
#endif // USE_DUAL_GYRO
	mpuDetect(&gyroSensor->gyroDev);
	mpuResetFn = gyroSensor->gyroDev.mpuConfiguration.resetFn; // must be set after mpuDetect
#endif
	gyroSensor->gyroDev.gyro_high_fsr = gyroConfig()->gyro_high_fsr;

	const gyroSensor_e gyroHardware = gyroDetect(&gyroSensor->gyroDev);
	if (gyroHardware == GYRO_NONE)
		return false;

	switch (gyroHardware) {
	case GYRO_MPU6500:
	case GYRO_MPU9250:
	case GYRO_ICM20601:
	case GYRO_ICM20602:
	case GYRO_ICM20608G:
	case GYRO_ICM20689:
		// do nothing, as gyro supports 32kHz
		break;
	default:
		// gyro does not support 32kHz
		gyroConfigMutable()->gyro_use_32khz = false;
		break;
	}

	// Must set gyro targetLooptime before gyroDev.init and initialisation of filters
	gyro.targetLooptime = gyroSetSampleRate(&gyroSensor->gyroDev,
			gyroConfig()->gyro_lpf, gyroConfig()->gyro_sync_denom,
			gyroConfig()->gyro_use_32khz);
	gyroSensor->gyroDev.lpf = gyroConfig()->gyro_lpf;
	gyroSensor->gyroDev.initFn(&gyroSensor->gyroDev);
	if (gyroConfig()->gyro_align != ALIGN_DEFAULT)
		gyroSensor->gyroDev.gyroAlign = gyroConfig()->gyro_align;

	gyroInitSensorFilters(gyroSensor);
#ifdef USE_GYRO_DATA_ANALYSE
	gyroDataAnalyseInit(gyro.targetLooptime);
#endif
	return true;
} // gyroInitSensor

bool gyroInit(void) {
	switch (debugMode) {
	case DEBUG_FFT:
	case DEBUG_GYRO:
	case DEBUG_GYRO_RAW:
		gyroDebugMode = debugMode;
		break;
	case DEBUG_GYRO_NOTCH:
	default:
		// debugMode is not gyro-related
		gyroDebugMode = DEBUG_NONE;
		break;
	}
	firstArmingCalibrationWasStarted = false;
	memset(&gyro, 0, sizeof(gyro));
	return gyroInitSensor(&gyroSensor1);
} // gyroInit

void gyroInitNewtonianLimiter(gyroSensor_t *gyroSensor) {

	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
		gyroSensor->gyroDev.gyroADCRawPrevious[axis] = 0;

} // gyroInitNewtonianLimiter

void gyroInitFilterLpf(gyroSensor_t *gyroSensor, uint8_t lpfHz) {

	const float gyroFrequencyNyquist = (1.0e6f / gyro.targetLooptime) * 0.5f;

	if ((lpfHz == 0) || (lpfHz > gyroFrequencyNyquist))
		lpfHz = gyroFrequencyNyquist;

	gyroSensor->softLpfFilterApplyFn = (filterApplyFnPtr) ptnFilterApply;
	const float gyroDt = (float) gyro.targetLooptime * 0.000001f;
	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		gyroSensor->softLpfFilterPtr[axis]
				= (filter_t *) &gyroSensor->softLpfFilter.gyroFilterPtnState[axis];
		ptnFilterInit(&gyroSensor->softLpfFilter.gyroFilterPtnState[axis], 2,
				lpfHz, gyroDt);
	}

} // gyroInitFilterLpf

static void gyroInitFilterKalman(gyroSensor_t *gyroSensor,
		uint16_t gyro_filter_q, uint16_t gyro_filter_r, uint16_t gyro_filter_p) {

	if (gyro_filter_q != 0 && gyro_filter_r != 0) {
		gyroSensor->fastKalmanApplyFn = (filterApplyFnPtr) fastKalmanUpdate;
		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
			fastKalmanInit(&gyroSensor->fastKalman[axis], gyro_filter_q,
					gyro_filter_r, gyro_filter_p);
	} else
		gyroSensor->fastKalmanApplyFn = nullFilterApply;
} // gyroInitFilterKalman

static void gyroInitFilterFixedKKalman(gyroSensor_t *gyroSensor, uint16_t lpfHz) {

	const float gyroFrequencyNyquist = (1.0e6f / gyro.targetLooptime) * 0.5f;

	if ((lpfHz == 0) || (lpfHz > gyroFrequencyNyquist))
		lpfHz = gyroFrequencyNyquist;

	gyroSensor->fixedKKalmanApplyFn = (filterApplyFnPtr) fixedKKalmanUpdate;
	const float gyroDt = (float) gyro.targetLooptime * 1.0e-6f;
	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
		fixedKKalmanInit(&gyroSensor->fastKalman[axis], lpfHz, gyroDt);

} // gyroInitFilterFixedKKalman


static void gyroInitSensorFilters(gyroSensor_t *gyroSensor) {

	gyroInitNewtonianLimiter(gyroSensor);
	gyroInitFilterKalman(gyroSensor, gyroConfig()->gyro_filter_q,
			gyroConfig()->gyro_filter_r, gyroConfig()->gyro_filter_p);
	gyroInitFilterFixedKKalman(gyroSensor, gyroConfig()->gyro_soft_lpf_hz_2);
	gyroInitFilterLpf(gyroSensor, gyroConfig()->gyro_soft_lpf_hz_2); // gke

} // gyroInitSensorFilters

void gyroInitFilters(void) {
	gyroInitSensorFilters(&gyroSensor1);
} // gyroInitFilters

FAST_CODE
bool isGyroSensorCalibrationComplete(const gyroSensor_t *gyroSensor) {
	return gyroSensor->calibration.calibratingG == 0;
} // isGyroSensorCalibrationComplete

FAST_CODE
bool isGyroCalibrationComplete(void) {
	return isGyroSensorCalibrationComplete(&gyroSensor1);
} // isGyroCalibrationComplete

static bool isOnFinalGyroCalibrationCycle(
		const gyroCalibration_t *gyroCalibration) {
	return gyroCalibration->calibratingG == 1;
} // isOnFinalGyroCalibrationCycle

static uint16_t gyroCalculateCalibratingCycles(void) {
	return (CALIBRATING_GYRO_CYCLES / gyro.targetLooptime)
			* CALIBRATING_GYRO_CYCLES;
} // gyroCalculateCalibratingCycles

static bool isOnFirstGyroCalibrationCycle(
		const gyroCalibration_t *gyroCalibration) {
	return gyroCalibration->calibratingG == gyroCalculateCalibratingCycles();
} // isOnFirstGyroCalibrationCycle

static void gyroSetCalibrationCycles(gyroSensor_t *gyroSensor) {
	gyroSensor->calibration.calibratingG = gyroCalculateCalibratingCycles();
} // gyroSetCalibrationCycles

void gyroStartCalibration(bool isFirstArmingCalibration) {
	if (!(isFirstArmingCalibration && firstArmingCalibrationWasStarted)) {
		gyroSetCalibrationCycles(&gyroSensor1);
		if (isFirstArmingCalibration)
			firstArmingCalibrationWasStarted = true;
	}
} // gyroStartCalibration

bool isFirstArmingGyroCalibrationRunning(void) {
	return firstArmingCalibrationWasStarted && !isGyroCalibrationComplete();
} // isFirstArmingGyroCalibrationRunning

STATIC_UNIT_TESTED void performGyroCalibration(gyroSensor_t *gyroSensor,
		uint8_t gyroMovementCalibrationThreshold) {
	for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
		// Reset g[axis] at start of calibration
		if (isOnFirstGyroCalibrationCycle(&gyroSensor->calibration)) {
			gyroSensor->calibration.sum[axis] = 0;
			devClear(&gyroSensor->calibration.var[axis]);
			// gyroZero is set to zero until calibration complete
			gyroSensor->gyroDev.gyroZero[axis] = 0;
		}

		// Sum up CALIBRATING_GYRO_CYCLES readings

		gyroSensor->calibration.sum[axis]
				+= gyroSensor->gyroDev.gyroADCRaw[axis];
		devPush(&gyroSensor->calibration.var[axis],
				gyroSensor->gyroDev.gyroADCRaw[axis]);

		if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
			const float stddev = devStandardDeviation(
					&gyroSensor->calibration.var[axis]);

			DEBUG_SET(DEBUG_GYRO, DEBUG_GYRO_CALIBRATION, lrintf(stddev));

			// check deviation and startover in case the model was moved
			if (gyroMovementCalibrationThreshold && stddev
					> gyroMovementCalibrationThreshold) {
				gyroSetCalibrationCycles(gyroSensor);
				return;
			}

			// please take care with exotic board alignment !!
			gyroSensor->gyroDev.gyroZero[axis]
					= gyroSensor->calibration.sum[axis]
							/ gyroCalculateCalibratingCycles();
			if (axis == Z)
				gyroSensor->gyroDev.gyroZero[axis]
						-= ((float) gyroConfig()->gyro_offset_yaw * 0.01f);
		}
	}

	if (isOnFinalGyroCalibrationCycle(&gyroSensor->calibration)) {
		schedulerResetTaskStatistics(TASK_SELF); // so calibration cycles do not pollute tasks statistics
		if (!firstArmingCalibrationWasStarted || (getArmingDisableFlags()
				& ~ARMING_DISABLED_CALIBRATING) == 0)
			beeper(BEEPER_GYRO_CALIBRATED);
	}

	--gyroSensor->calibration.calibratingG;

} // performGyroCalibration

#if defined(ROBERT)

FAST_CODE int32_t SensorSlewLimit(int32_t * Old, int32_t New, int32_t Slew) {
	int32_t Low, High;

	Low = *Old - Slew;
	High = *Old + Slew;
	if (New < Low)
	*Old = Low;
	else if (New > High)
	*Old = High;
	else
	*Old = New;
	return (*Old);
} // SensorSlewLimit

FAST_CODE int32_t gyroSlewLimiter(gyroSensor_t *gyroSensor, int axis) {
	static uint32_t SlewLimitGyroClicks = 128;
	static int32_t BP[3] = {0};

	int32_t ret = (int32_t)gyroSensor->gyroDev.gyroADCRaw[axis];
	if (gyroConfig()->checkOverflow)
	// don't use the slew limiter if overflow checking is on
	return ret;

	return SensorSlewLimit(&BP[axis], ret, SlewLimitGyroClicks);;
} // gyroSlewLimiter

FAST_CODE int32_t gyroNewtonianLimiter(gyroSensor_t *gyroSensor, int axis) {
	return gyroSlewLimiter(gyroSensor, axis);
} // gyroNewtonianLimiter

#else

FAST_CODE
int32_t gyroNewtonianLimiter(gyroSensor_t *gyroSensor, int axis) {
	// can be generalised to filtering max acceptable physical change - slewlimit
	// this version specific to gyro wraparound
	int32_t ret = (int32_t) gyroSensor->gyroDev.gyroADCRaw[axis]; // extend to 32bits

	if (abs(ret - gyroSensor->gyroDev.gyroADCRawPrevious[axis]) > (1 << 14))
		// there has been a large change in value, so assume overflow has occurred and return the previous value
		ret = gyroSensor->gyroDev.gyroADCRawPrevious[axis];
	else
		gyroSensor->gyroDev.gyroADCRawPrevious[axis] = ret;

	return ret;
} // gyroNewtonianLimiter

#endif

static FAST_CODE void gyroUpdateSensor(gyroSensor_t *gyroSensor,
		timeUs_t currentTimeUs) {
#ifdef USE_DMA_SPI_DEVICE
	if (dmaSpiGyroDataReady)
#else
	if (gyroSensor->gyroDev.readFn(&gyroSensor->gyroDev))
#endif
	{
		gyroSensor->gyroDev.dataReady = false;

		if (isGyroSensorCalibrationComplete(gyroSensor)) {
			// move 16-bit gyro data into 32-bit variables
			for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
				gyroSensor->gyroDev.gyroADC[axis] = gyroNewtonianLimiter(
						gyroSensor, axis) - gyroSensor->gyroDev.gyroZero[axis];

			alignSensors(gyroSensor->gyroDev.gyroADC,
					gyroSensor->gyroDev.gyroAlign);
		} else {
			performGyroCalibration(gyroSensor,
					gyroConfig()->gyroMovementCalibrationThreshold);
			// Reset gyro values to zero to prevent other code from using uncalibrated data
			for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
				gyro.gyroADCf[axis] = 0.0f;

			// still calibrating, so no need to further process gyro data
			return;
		}

		const timeDelta_t sampleDeltaUs = currentTimeUs
				- accumulationLastTimeSampledUs;
		accumulationLastTimeSampledUs = currentTimeUs;
		accumulatedMeasurementTimeUs += sampleDeltaUs;

		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			if (gyroDebugMode != DEBUG_NONE)
				DEBUG_SET(DEBUG_GYRO_RAW, axis, gyroSensor->gyroDev.gyroADCRaw[axis]);

			// scale gyro output to degrees per second
			float gyroADCf = gyroSensor->gyroDev.gyroADC[axis]
					* gyroSensor->gyroDev.scale;

			switch (gyroConfig()->gyro_stage2_filter_type) {
			case STAGE2_FILTER_FAST_KALMAN: // Kalyn fast KF
				gyroADCf = gyroSensor->fastKalmanApplyFn(
						(filter_t *) &gyroSensor->fastKalman[axis], gyroADCf);
				break;
			case STAGE2_FILTER_FIXED_K_KALMAN: // Fujin Fixed K KF
				gyroADCf = gyroSensor->fixedKKalmanApplyFn(
						(filter_t *) &gyroSensor->fastKalman[axis], gyroADCf);
				break;
			case STAGE2_FILTER_PTn: // Simple order n filter
				gyroADCf = gyroSensor->softLpfFilterApplyFn(
						gyroSensor->softLpfFilterPtr[axis], gyroADCf);
				break;
			default:
				break;
			} // switch

			if (gyroDebugMode != DEBUG_NONE)
				DEBUG_SET(DEBUG_GYRO, axis, lrintf(gyroADCf));

			gyro.gyroADCf[axis] = gyroADCf;
			// trapezoidal integration to avoid bias
			accumulatedMeasurements[axis] += 0.5f * (previousgyroADCf[axis]
					+ gyroADCf) * sampleDeltaUs;
			previousgyroADCf[axis] = gyroADCf;
		}
	}
} // gyroUpdateSensor

#ifdef USE_DMA_SPI_DEVICE
FAST_CODE void gyroDmaSpiFinishRead(void) {
	//called by dma callback
	mpuGyroDmaSpiReadFinish(&gyroSensor1.gyroDev);
} // gyroDmaSpiFinishRead

FAST_CODE void gyroDmaSpiStartRead(void) {
	//called by scheduler
	gyroSensor1.gyroDev.readFn(&gyroSensor1.gyroDev);
} // gyroDmaSpiStartRead
#endif

FAST_CODE
void gyroUpdate(timeUs_t currentTimeUs) {
	//called by scheduler
	gyroUpdateSensor(&gyroSensor1, currentTimeUs);
} // gyroUpdate

bool gyroGetAccumulationAverage(float *accumulationAverage) {
	if (accumulatedMeasurementTimeUs > 0) {
		// If we have gyro data accumulated, calculate average rate that will yield the same rotation
		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++) {
			accumulationAverage[axis] = accumulatedMeasurements[axis]
					/ accumulatedMeasurementTimeUs;
			accumulatedMeasurements[axis] = 0.0f;
		}
		accumulatedMeasurementTimeUs = 0;
		return true;
	} else {
		for (int axis = 0; axis < XYZ_AXIS_COUNT; axis++)
			accumulationAverage[axis] = 0.0f;
		return false;
	}
} // gyroGetAccumulationAverage


// Misc

void gyroReadTemperature(void) {
	if (gyroSensor1.gyroDev.temperatureFn)
		gyroSensor1.gyroDev.temperatureFn(&gyroSensor1.gyroDev,
				&gyroSensor1.gyroDev.temperature);
} // gyroReadTemperature

int16_t gyroGetTemperature(void) {
	return gyroSensor1.gyroDev.temperature;
} // gyroGetTemperature

int16_t gyroRateDps(int axis) {
	return lrintf(gyro.gyroADCf[axis] / gyroSensor1.gyroDev.scale);
} // gyroRateDps

uint16_t gyroAbsRateDps(int axis) {
	return fabsf(gyro.gyroADCf[axis]);
} // gyroAbsRateDps


