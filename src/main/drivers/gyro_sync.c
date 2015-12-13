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
static uint8_t gyroFilterRate;

bool getMpuDataStatus(gyro_t *gyro)
{
    bool mpuDataStatus;

    gyro->intStatus(&mpuDataStatus);
    return mpuDataStatus;
}

bool gyroSyncCheckUpdate(void) {
    return getMpuDataStatus(&gyro);
}

void gyroUpdateSampleRate(void) {

    int gyroFrequency;
    int gyroSampleRate;
#if defined(REVONANO)
    gyroFrequency  = 31;   // gyro sampling rate 8khz
    gyroSampleRate = 31; // 8khz sampling
    targetLooptime = 250;  // Wanted looptime
#else
    gyroFrequency  = 125;   // gyro sampling rate 8khz
    gyroSampleRate = 125; // 8khz sampling
    targetLooptime = 250;  // Wanted looptime
#endif

//    gyroFrequency  = 1000;   // gyro sampling rate 1khz
//    gyroSampleRate = 1000; // 1khz sampling
//    targetLooptime = 1000;  // Wanted looptime

    // calculate gyro divider and targetLooptime (expected cycleTime)
    mpuDividerDrops = ( gyroSampleRate / gyroFrequency ) - 1;
    gyroFilterRate  = ( targetLooptime / gyroSampleRate );

}

uint8_t gyroMPU6xxxGetDividerDrops(void) {
    return mpuDividerDrops;
}
