/*
 * gyro_sync.h
 *
 *  Created on: 3 aug. 2015
 *      Author: borisb
 */

#define INTERRUPT_WAIT_TIME 3
extern uint32_t targetLooptime;
extern uint32_t targetESCwritetime;

extern uint8_t ESCWriteDenominator;

bool gyroSyncCheckUpdate(void);
uint8_t gyroMPU6xxxGetDividerDrops(void);
void gyroUpdateSampleRate(uint8_t lpf, uint8_t gyroSyncDenominator);
