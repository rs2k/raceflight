/*
 * gyro_sync.c
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/axis.h"
#include "common/maths.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/gyro_sync.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "config/runtime_config.h"
#include "config/config.h"

extern gyro_t gyro;

uint32_t targetLooptime;
static uint8_t mpuDividerDrops;

bool getMpuDataStatus(gyro_t *gyro)
{
    bool mpuDataStatus;

    gyro->intStatus(&mpuDataStatus);
    return mpuDataStatus;
}

bool gyroSyncCheckUpdate(void) {
    return getMpuDataStatus(&gyro);
}

void gyroUpdateSampleRate(uint8_t lpf) {

    int gyroSamplePeriod, gyroSyncDenominator;

#if defined(COLIBRI_RACE)
    if (lpf == 4) {
    	gyroSamplePeriod = 1000;
    	gyroSyncDenominator = 1; // Full Sampling 1khz
    } else {
    	gyroSamplePeriod = 125;
    	gyroSyncDenominator = 3; // Sample every 3d gyro measurement 2,6khz
    }
#elif defined(STM32F303xC)
    if (lpf == 4) {
    	gyroSamplePeriod = 1000;
    	gyroSyncDenominator = 1; // Full Sampling 1khz
    } else {
    	gyroSamplePeriod = 125;
    	gyroSyncDenominator = 4; // Sample every 8th gyro measurement 1khz
    }
#elif defined(REVONANO) || defined(SPARKY2) || defined(ALIENFLIGHTF4) || defined(BLUEJAYF4) || defined(VRCORE)
	if (lpf == 0) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every 4th gyro measurement 1khz
	} else  if (lpf == 1) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every 4th gyro measurement 2khz
	} else  if (lpf == 2) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every 2nd gyro measurement 4khz
	} else  if (lpf == 3) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every gyro measurement 8khz
	} else if (lpf == 4) {
		gyroSamplePeriod = 1000;
		gyroSyncDenominator = 1; // Full Sampling 1khz
	} else if (lpf == 5) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every 4th gyro measurement 1khz
	} else  if (lpf == 6) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every 4th gyro measurement 2khz
	} else  if (lpf == 7) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every 2nd gyro measurement 4khz
	} else  if (lpf == 8) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every gyro measurement 8khz
	}
#elif defined(REVO)
	if (lpf == 0) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 8; // Sample every 4th gyro measurement 1khz
	} else  if (lpf == 1) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 4; // Sample every 4th gyro measurement 2khz
	} else  if (lpf == 2) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 2; // Sample every 2nd gyro measurement 4khz
	} else  if (lpf == 3) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every gyro measurement 8khz
	} else if (lpf == 4) {
		gyroSamplePeriod = 1000;
		gyroSyncDenominator = 1; // Full Sampling 1khz
	} else if (lpf == 5) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 8; // Sample every 4th gyro measurement 1khz
	} else  if (lpf == 6) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 4; // Sample every 4th gyro measurement 2khz
	} else  if (lpf == 7) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 2; // Sample every 2nd gyro measurement 4khz
	} else  if (lpf == 8) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 1; // Sample every gyro measurement 8khz
	}
#else
	if (!sensors(SENSOR_ACC) && !sensors(SENSOR_BARO) && !sensors(SENSOR_MAG)) {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 4; // Sample every 4th gyro measurement 2khz
	} else if (lpf == 4) {
		gyroSamplePeriod = 1000;
		gyroSyncDenominator = 1; // Full Sampling 1khz
	} else {
		gyroSamplePeriod = 125;
		gyroSyncDenominator = 8; // Sample every 8th gyro measurement 1khz
	}
#endif

    // calculate gyro divider and targetLooptime (expected cycleTime)
    mpuDividerDrops  = gyroSyncDenominator - 1;
    targetLooptime = (mpuDividerDrops + 1) * gyroSamplePeriod;

}

uint8_t gyroMPU6xxxGetDividerDrops(void) {
    return mpuDividerDrops;
}
