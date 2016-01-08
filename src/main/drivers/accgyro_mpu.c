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
#include <stdlib.h>
#include <string.h>

#include "platform.h"
#include "build_config.h"
#include "debug.h"

#include "common/maths.h"

#include "nvic.h"

#include "system.h"
#include "gpio.h"
#include "exti.h"
#include "bus_i2c.h"
#include "gyro_sync.h"

#include "sensor.h"
#include "accgyro.h"
#include "accgyro_mpu3050.h"
#include "accgyro_mpu6050.h"
#include "accgyro_mpu6500.h"
#include "accgyro_spi_mpu6000.h"
#include "accgyro_spi_mpu6500.h"
#include "accgyro_spi_mpu9250.h"
#include "accgyro_mpu.h"

#define DEBUG_MPU_DATA_READY_INTERRUPT

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t* data);
static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data);

static void mpu6050FindRevision(void);

static int gyro_i_count = 0;
static volatile bool mpuDataReady;
static bool filterFull = false;
static int gyroADCnums = 0;
#if defined(REVONANO) || defined(SPARKY2) || defined(ALIENFLIGHTF4) || defined(BLUEJAYF4) || defined(VRCORE)
//#define gyroFilterLevel 8 //todo move to gyro_sync and calculate.
#define gyroFilterLevel 2 //todo move to gyro_sync and calculate.
#else
#define gyroFilterLevel 2 //todo move to gyro_sync and calculate.
#endif
static int16_t gyroADCtable0[gyroFilterLevel];
static int16_t gyroADCtable1[gyroFilterLevel];
static int16_t gyroADCtable2[gyroFilterLevel];
static int32_t gyroTotal0 = 0;
static int32_t gyroTotal1 = 0;
static int32_t gyroTotal2 = 0;

static int32_t timewatch[32];
static int timewatchdog = 0;


#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(void);
#endif

mpuDetectionResult_t mpuDetectionResult;

mpuConfiguration_t mpuConfiguration;
static const extiConfig_t *mpuIntExtiConfig = NULL;

#define MPU_ADDRESS             0x68

// WHO_AM_I register contents for MPU3050, 6050 and 6500
#define MPU6500_WHO_AM_I_CONST              (0x70)
#define MPUx0x0_WHO_AM_I_CONST              (0x68)

#define MPU_INQUIRY_MASK   0x7E

mpuDetectionResult_t *detectMpu(const extiConfig_t *configToUse)
{
    memset(&mpuDetectionResult, 0, sizeof(mpuDetectionResult));
    memset(&mpuConfiguration, 0, sizeof(mpuConfiguration));

    mpuIntExtiConfig = configToUse;

    bool ack;
    uint8_t sig;
    uint8_t inquiryResult;

    // MPU datasheet specifies 30ms.
    delay(35);

#ifndef USE_I2C
    ack = false;
    sig = 0;
#else
    ack = mpuReadRegisterI2C(MPU_RA_WHO_AM_I, 1, &sig);
#endif
    if (ack) {
    	mpuConfiguration.read = mpuReadRegisterI2C;
		mpuConfiguration.write = mpuWriteRegisterI2C;
		mpuConfiguration.slowread = mpuReadRegisterI2C;
		mpuConfiguration.verifywrite = mpuWriteRegisterI2C;
    } else {
#ifdef USE_SPI
        bool detectedSpiSensor = detectSPISensorsAndUpdateDetectionResult();
        UNUSED(detectedSpiSensor);
#endif

        return &mpuDetectionResult;
    }

    mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;

    // If an MPU3050 is connected sig will contain 0.
    ack = mpuReadRegisterI2C(MPU_RA_WHO_AM_I_LEGACY, 1, &inquiryResult);
    inquiryResult &= MPU_INQUIRY_MASK;
    if (ack && inquiryResult == MPUx0x0_WHO_AM_I_CONST) {
        mpuDetectionResult.sensor = MPU_3050;
        mpuConfiguration.gyroReadXRegister = MPU3050_GYRO_OUT;
        return &mpuDetectionResult;
    }

    sig &= MPU_INQUIRY_MASK;

    if (sig == MPUx0x0_WHO_AM_I_CONST) {

        mpuDetectionResult.sensor = MPU_60x0;

        mpu6050FindRevision();
    } else if (sig == MPU6500_WHO_AM_I_CONST) {
        mpuDetectionResult.sensor = MPU_65xx_I2C;
    }

    return &mpuDetectionResult;
}

#ifdef USE_SPI
static bool detectSPISensorsAndUpdateDetectionResult(void)
{
#ifdef USE_GYRO_SPI_MPU6500
    if (mpu6500SpiDetect()) {
        mpuDetectionResult.sensor = MPU_65xx_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.read = mpu6500ReadRegister;
        mpuConfiguration.slowread = mpu6500SlowReadRegister;
        mpuConfiguration.verifywrite = verifympu6500WriteRegister;
        mpuConfiguration.write = mpu6500WriteRegister;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_MPU6000
    if (mpu6000SpiDetect()) {
        mpuDetectionResult.sensor = MPU_60x0_SPI;
        mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
        mpuConfiguration.read = mpu6000ReadRegister;
        mpuConfiguration.slowread = mpu6000SlowReadRegister;
        mpuConfiguration.verifywrite = verifympu6000WriteRegister;
        mpuConfiguration.write = mpu6000WriteRegister;
        return true;
    }
#endif

#ifdef USE_GYRO_SPI_MPU9250
	if (mpu9250SpiDetect()) {
		mpuDetectionResult.sensor = MPU_9250_SPI;
		mpuConfiguration.gyroReadXRegister = MPU_RA_GYRO_XOUT_H;
		mpuConfiguration.read = mpu9250ReadRegister;
        mpuConfiguration.slowread = mpu9250SlowReadRegister;
        mpuConfiguration.verifywrite = verifympu9250WriteRegister;
		mpuConfiguration.write = mpu9250WriteRegister;
		return true;
	}
#endif

    return false;
}
#endif

static void mpu6050FindRevision(void)
{
    bool ack;
    UNUSED(ack);

    uint8_t readBuffer[6];
    uint8_t revision;
    uint8_t productId;

    // There is a map of revision contained in the android source tree which is quite comprehensive and may help to understand this code
    // See https://android.googlesource.com/kernel/msm.git/+/eaf36994a3992b8f918c18e4f7411e8b2320a35f/drivers/misc/mpu6050/mldl_cfg.c

    // determine product ID and accel revision
    ack = mpuConfiguration.read(MPU_RA_XA_OFFS_H, 6, readBuffer);
    revision = ((readBuffer[5] & 0x01) << 2) | ((readBuffer[3] & 0x01) << 1) | (readBuffer[1] & 0x01);
    if (revision) {
        /* Congrats, these parts are better. */
        if (revision == 1) {
            mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else if (revision == 2) {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        } else {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        }
    } else {
        ack = mpuConfiguration.read(MPU_RA_PRODUCT_ID, 1, &productId);
        revision = productId & 0x0F;
        if (!revision) {
            failureMode(FAILURE_ACC_INCOMPATIBLE);
        } else if (revision == 4) {
            mpuDetectionResult.resolution = MPU_HALF_RESOLUTION;
        } else {
            mpuDetectionResult.resolution = MPU_FULL_RESOLUTION;
        }
    }
}

void MPU_DATA_READY_EXTI_Handler(void)
{

#if defined (REVO)
    if (EXTI_GetITStatus(EXTI_Line4) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line4);
#elif defined (REVONANO)
    if (EXTI_GetITStatus(EXTI_Line15) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line15);
#elif defined (SPARKY2) || defined(BLUEJAYF4)
    if (EXTI_GetITStatus(EXTI_Line5) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line5);
#elif defined (ALIENFLIGHTF4)
    if (EXTI_GetITStatus(EXTI_Line14) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line14);
#elif defined (VRCORE)
    if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
        EXTI_ClearITPendingBit(EXTI_Line10);
#else
    if (EXTI_GetITStatus(mpuIntExtiConfig->exti_line) != RESET) {
        EXTI_ClearITPendingBit(mpuIntExtiConfig->exti_line);
#endif

#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
		static uint32_t lastCalledAt = 0;
		uint32_t now = micros();
        uint32_t callDelta = now - lastCalledAt;
		debug[0] = callDelta;
		lastCalledAt = now;
		if (callDelta > 150) {
			if (timewatchdog < 32) {
				timewatch[timewatchdog] = 1;
				timewatchdog++;
			}
		} else {
			if (timewatchdog < 32) {
				timewatch[timewatchdog] = 0;
				timewatchdog++;
			}
		}
		if (timewatchdog == 32) {
			debug[2]=timewatch[0] << 0 | timewatch[1] << 1 | timewatch[2] << 2  | timewatch[3] << 3  | timewatch[4] << 4  | timewatch[5] << 5  | timewatch[6] << 6  | timewatch[7] << 7 | timewatch[8] << 8 | timewatch[9] << 9 | timewatch[10]<< 10 | timewatch[11]<< 11 | timewatch[12]<< 12 | timewatch[13]<< 13 | timewatch[14]<< 14 | timewatch[15]<< 15;
			debug[3]=timewatch[16]<< 0 | timewatch[17]<< 1 | timewatch[18]<< 2  | timewatch[19]<< 3  | timewatch[20]<< 4  | timewatch[21]<< 5  | timewatch[22]<< 6  | timewatch[23]<< 7 | timewatch[24]<< 8 | timewatch[25]<< 9 | timewatch[26]<< 10 | timewatch[27]<< 11 | timewatch[28]<< 12 | timewatch[29]<< 13 | timewatch[30]<< 14 | timewatch[31]<< 15;
			debug[1]++;
			timewatchdog=0;
		}
#endif


		gyro_i_count++;
		mpuGyroReadCollect();

		if (gyro_i_count >= gyroFilterLevel) {


#ifdef DEBUG_MPU_DATA_READY_INTERRUPT
			static uint32_t lastCalledAt1 = 0;
			uint32_t now1 = micros();
	        uint32_t callDelta1 = now1 - lastCalledAt1;
	        debug[1] = callDelta1;
    		lastCalledAt1 = now;
#endif
			gyro_i_count = 0;
    		mpuDataReady = true;
		}

    }

}


void configureMPUDataReadyInterruptHandling(void)
{
#if defined(USE_MPU_DATA_READY_SIGNAL) && defined(REVO)

	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource4);

	EXTI_InitStruct.EXTI_Line = EXTI_Line4;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI4_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	registerExtiCallbackHandler(EXTI4_IRQn, MPU_DATA_READY_EXTI_Handler);

	EXTI_ClearITPendingBit(EXTI_Line4);

#elif defined(USE_MPU_DATA_READY_SIGNAL) && defined(SPARKY2)

    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);

    EXTI_InitStruct.EXTI_Line = EXTI_Line5;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    registerExtiCallbackHandler(EXTI9_5_IRQn, MPU_DATA_READY_EXTI_Handler);

    EXTI_ClearITPendingBit(EXTI_Line5);
    
#elif defined(USE_MPU_DATA_READY_SIGNAL) && defined(BLUEJAYF4)

    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource5);

    EXTI_InitStruct.EXTI_Line = EXTI_Line5;
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_Init(&EXTI_InitStruct);

    NVIC_InitStruct.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStruct);

    registerExtiCallbackHandler(EXTI9_5_IRQn, MPU_DATA_READY_EXTI_Handler);

    EXTI_ClearITPendingBit(EXTI_Line5);

#elif defined(USE_MPU_DATA_READY_SIGNAL) && defined(ALIENFLIGHTF4)

    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource14);

    /* PD0 is connected to EXTI_Line0 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line14;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
    /* Set priority */
    NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
    /* Set sub priority */
    NVIC_InitStruct.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
    /* Enable interrupt */
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);

    registerExtiCallbackHandler(EXTI15_10_IRQn, MPU_DATA_READY_EXTI_Handler);

    EXTI_ClearITPendingBit(EXTI_Line14);

#elif defined(USE_MPU_DATA_READY_SIGNAL) && defined(REVONANO)

	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_15;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource15);

	EXTI_InitStruct.EXTI_Line = EXTI_Line15;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	registerExtiCallbackHandler(EXTI15_10_IRQn, MPU_DATA_READY_EXTI_Handler);

	EXTI_ClearITPendingBit(EXTI_Line15);

#elif defined(USE_MPU_DATA_READY_SIGNAL) && defined(VRCORE)

	GPIO_InitTypeDef GPIO_InitStruct;
	EXTI_InitTypeDef EXTI_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);

	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource10);

	EXTI_InitStruct.EXTI_Line = EXTI_Line10;
	EXTI_InitStruct.EXTI_LineCmd = ENABLE;
	EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_Init(&EXTI_InitStruct);

	NVIC_InitStruct.NVIC_IRQChannel = EXTI15_10_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);

	registerExtiCallbackHandler(EXTI15_10_IRQn, MPU_DATA_READY_EXTI_Handler);

	EXTI_ClearITPendingBit(EXTI_Line10);


#elif defined(USE_MPU_DATA_READY_SIGNAL)

#ifdef STM32F10X
    // enable AFIO for EXTI support
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
#endif

#ifdef STM32F303xC
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

#if defined(STM32F40_41xxx) || defined (STM32F411xE)
    /* Enable SYSCFG clock otherwise the EXTI irq handlers are not called */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif

#ifdef STM32F10X
    gpioExtiLineConfig(mpuIntExtiConfig->exti_port_source, mpuIntExtiConfig->exti_pin_source);
#endif

#ifdef STM32F303xC
    gpioExtiLineConfig(mpuIntExtiConfig->exti_port_source, mpuIntExtiConfig->exti_pin_source);
#endif

#if defined(STM32F40_41xxx) || defined (STM32F411xE)
    gpioExtiLineConfig(mpuIntExtiConfig->exti_port_source, mpuIntExtiConfig->exti_pin_source);
#endif

#ifdef ENSURE_MPU_DATA_READY_IS_LOW
    uint8_t status = GPIO_ReadInputDataBit(mpuIntExtiConfig->gpioPort, mpuIntExtiConfig->gpioPin);
    if (status) {
        return;
    }
#endif

    registerExtiCallbackHandler(mpuIntExtiConfig->exti_irqn, MPU_DATA_READY_EXTI_Handler);

    EXTI_ClearITPendingBit(mpuIntExtiConfig->exti_line);

    EXTI_InitTypeDef EXTIInit;
    EXTIInit.EXTI_Line = mpuIntExtiConfig->exti_line;
    EXTIInit.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTIInit.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTIInit.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTIInit);

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = mpuIntExtiConfig->exti_irqn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_MPU_DATA_READY);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
}

void mpuIntExtiInit(void)
{
    gpio_config_t gpio;

    static bool mpuExtiInitDone = false;

    if (mpuExtiInitDone || !mpuIntExtiConfig) {
        return;
    }

#if defined(STM32F40_41xxx) || defined (STM32F411xE)
        if (mpuIntExtiConfig->gpioAHB1Peripherals) {
            RCC_AHB1PeriphClockCmd(mpuIntExtiConfig->gpioAHB1Peripherals, ENABLE);
        }
#endif
#ifdef STM32F303
        if (mpuIntExtiConfig->gpioAHBPeripherals) {
            RCC_AHBPeriphClockCmd(mpuIntExtiConfig->gpioAHBPeripherals, ENABLE);
        }
#endif
#ifdef STM32F10X
        if (mpuIntExtiConfig->gpioAPB2Peripherals) {
            RCC_APB2PeriphClockCmd(mpuIntExtiConfig->gpioAPB2Peripherals, ENABLE);
        }
#endif

    gpio.pin = mpuIntExtiConfig->gpioPin;
    gpio.speed = Speed_2MHz;
    gpio.mode = Mode_IN_FLOATING;
    gpioInit(mpuIntExtiConfig->gpioPort, &gpio);

    configureMPUDataReadyInterruptHandling();

    mpuExtiInitDone = true;
}

static bool mpuReadRegisterI2C(uint8_t reg, uint8_t length, uint8_t* data)
{
    bool ack = i2cRead(MPU_ADDRESS, reg, length, data);
    return ack;
}

static bool mpuWriteRegisterI2C(uint8_t reg, uint8_t data)
{
    bool ack = i2cWrite(MPU_ADDRESS, reg, data);
    return ack;
}

bool mpuAccRead(int16_t *accData)
{
    uint8_t data[6];

    __disable_irq();
    bool ack = mpuConfiguration.read(MPU_RA_ACCEL_XOUT_H, 6, data);
    __enable_irq();
    if (!ack) {
        return false;
    }

    accData[0] = (int16_t)((data[0] << 8) | data[1]);
    accData[1] = (int16_t)((data[2] << 8) | data[3]);
    accData[2] = (int16_t)((data[4] << 8) | data[5]);

    return true;
}

bool mpuGyroReadCollect(void)
{
	if (gyroFilterLevel == 1) {
		return true; //no filtering or downsampling needed
	}
    uint8_t data[6];

    bool ack = mpuConfiguration.read(mpuConfiguration.gyroReadXRegister, 6, data);
    if (!ack) {
        return false;
    }

    gyroTotal0 -= gyroADCtable0[gyroADCnums];
    gyroTotal1 -= gyroADCtable1[gyroADCnums];
    gyroTotal2 -= gyroADCtable2[gyroADCnums];

    gyroADCtable0[gyroADCnums] = (int16_t)((data[0] << 8) | data[1]);
    gyroADCtable1[gyroADCnums] = (int16_t)((data[2] << 8) | data[3]);
    gyroADCtable2[gyroADCnums] = (int16_t)((data[4] << 8) | data[5]);

    gyroTotal0 += gyroADCtable0[gyroADCnums];
    gyroTotal1 += gyroADCtable1[gyroADCnums];
    gyroTotal2 += gyroADCtable2[gyroADCnums];

    gyroADCnums++;
    if (gyroADCnums >= gyroFilterLevel) {
    	gyroADCnums = 0;
    	filterFull = true;
    }

    return true;
}

bool mpuGyroRead(int16_t *gyroADC)
{
	if ( !filterFull ) {

	    uint8_t data[6];

	    bool ack = mpuConfiguration.read(mpuConfiguration.gyroReadXRegister, 6, data);
	    if (!ack) {
	        return false;
	    }

	    gyroADC[0] = (int16_t)((data[0] << 8) | data[1]);
		gyroADC[1] = (int16_t)((data[2] << 8) | data[3]);
		gyroADC[2] = (int16_t)((data[4] << 8) | data[5]);

	} else {

		gyroADC[0] = (int16_t)( ( gyroTotal0 ) / gyroFilterLevel);
		gyroADC[1] = (int16_t)( ( gyroTotal1 ) / gyroFilterLevel);
		gyroADC[2] = (int16_t)( ( gyroTotal2 ) / gyroFilterLevel);

	}

    return true;
}

void checkMPUDataReady(bool *mpuDataReadyPtr) {
    if (mpuDataReady) {
        *mpuDataReadyPtr = true;
        mpuDataReady= false;
    } else {
        *mpuDataReadyPtr = false;
    }
}
