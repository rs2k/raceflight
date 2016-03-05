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
uint32_t targetESCwritetime;

static uint8_t mpuDividerDrops;

uint8_t ESCWriteDenominator;
static bool mpuDividerDropsOverride;

bool getMpuDataStatus(gyro_t *gyro)
{
    bool mpuDataStatus;

    gyro->intStatus(&mpuDataStatus);
    return mpuDataStatus;
}

bool gyroSyncCheckUpdate(void) {
    return getMpuDataStatus(&gyro);
}

void gyroUpdateSampleRate(uint8_t lpf, uint8_t gyroSyncDenominator) {
    int gyroSamplePeriod;

#if defined(COLIBRI_RACE)
    switch (lpf) {
		case 0:
        	gyroSamplePeriod = 125;
    		gyroSyncDenominator = 8; // Sample every 8th gyro measurement 1khz
    		ESCWriteDenominator = 1; // ESC Write at 1khz
    		mpuDividerDropsOverride = false; // override mpuDividerDrops
    		break;
		case 1:
		case 2:
		case 3:
        	gyroSamplePeriod = 125;
    		gyroSyncDenominator = 4; // Sample every 4th gyro measurement 2khz
    		ESCWriteDenominator = 1; // ESC Write at 2khz
    		mpuDividerDropsOverride = false; // override mpuDividerDrops
    		break;
        case 4:
        	gyroSamplePeriod = 1000;
        	gyroSyncDenominator = 1; // Full Sampling 1khz
    		ESCWriteDenominator = 1; // ESC Write at 1khz
    		mpuDividerDropsOverride = false; // do not override mpuDividerDrops
        	break;
        default:
        	gyroSamplePeriod = 125;
        	gyroSyncDenominator = 3; // Sample every 3d gyro measurement 2.6khz
    		ESCWriteDenominator = 1; // ESC Write at 2.6khz
    		mpuDividerDropsOverride = false; // do not override mpuDividerDrops
            break;
    }
#elif defined(STM32F303xC)
    switch (lpf) {
		case 0:
        	gyroSamplePeriod = 125;
    		gyroSyncDenominator = 8; // Sample every gyro measurement 1khz
    		ESCWriteDenominator = 1; // ESC Write at 1khz
    		mpuDividerDropsOverride = false; // override mpuDividerDrops
    		break;
		case 1:
		case 2:
		case 3:
        	gyroSamplePeriod = 125;
    		gyroSyncDenominator = 4; // Sample every 4th gyro measurement 2khz
        	ESCWriteDenominator = 1; // ESC Write at 2khz
    		mpuDividerDropsOverride = false; // override mpuDividerDrops
    		break;
        case 4:
        	gyroSamplePeriod = 1000;
        	gyroSyncDenominator = 1; // Full Sampling 1khz
        	ESCWriteDenominator = 1; // ESC Write at 1khz
    		mpuDividerDropsOverride = false; // do not override mpuDividerDrops
    		break;
        default:
        	gyroSamplePeriod = 125;
    		gyroSyncDenominator = 8; // Sample every 8th gyro measurement 1khz
        	ESCWriteDenominator = 1; // ESC Write at 1khz
    		mpuDividerDropsOverride = false; // do not override mpuDividerDrops
           break;
    }
#elif defined(STM32F40_41xxx) || defined(STM32F411xE)
    switch (lpf) {
		case 0:
		case 1:
		case 2:
		case 3:
        	gyroSamplePeriod = 125;
    		gyroSyncDenominator = 1; // Sample every gyro measurement 8khz
    		mpuDividerDropsOverride = true; // override mpuDividerDrops
    		break;
		case 4:
        	gyroSamplePeriod = 1000;
        	gyroSyncDenominator = 1; // Full Sampling 1khz
    		mpuDividerDropsOverride = false; // do not override mpuDividerDrops
    		break;
		case 5:
		case 6:
		case 7:
		case 8:
        	gyroSamplePeriod = 125;
    		gyroSyncDenominator = 1; // Sample every gyro measurement 8khz
    		mpuDividerDropsOverride = true; // do not override mpuDividerDrops
    		break;
    }
    switch (lpf) {
		case 0:
		case 5:
    		ESCWriteDenominator = 8; // ESC Write at 1khz
    		break;
		case 1:
		case 6:
    		ESCWriteDenominator = 4; // ESC Write at 2khz
    		break;
		case 2:
		case 7:
    		ESCWriteDenominator = 2; // ESC Write at 4khz
    		break;
		case 3:
		case 8:
    		ESCWriteDenominator = 1; // ESC Write at 8khz
    		break;
        case 4:
    		ESCWriteDenominator = 1; // ESC Write at 8khz
    		break;
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
	ESCWriteDenominator = 1;
	mpuDividerDropsOverride = false;
#endif

    // calculate gyro divider and targetLooptime (expected cycleTime)
    mpuDividerDrops  = gyroSyncDenominator - 1;
    targetLooptime = (mpuDividerDrops + 1) * gyroSamplePeriod;
    targetESCwritetime = gyroSamplePeriod*ESCWriteDenominator;

}

uint8_t gyroMPU6xxxGetDividerDrops(void) {
	if (mpuDividerDropsOverride) {
		return 0;
	}
    return mpuDividerDrops;
}
