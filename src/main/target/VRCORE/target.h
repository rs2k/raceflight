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

#pragma once
#define TARGET_BOARD_IDENTIFIER "VRCORE"

#define LED0_GPIO   GPIOD
#define LED0_PIN    Pin_14 // Blue LEDs - PB5
#define LED0_PERIPHERAL RCC_AHB1Periph_GPIOD
#define LED1_GPIO   GPIOD
#define LED1_PIN    Pin_15  // Red LEDs - PB4
#define LED1_PERIPHERAL RCC_AHB1Periph_GPIOD

#define BEEP_GPIO   GPIOA
#define BEEP_PIN    Pin_0
#define BEEP_PERIPHERAL RCC_AHB1Periph_GPIOA

#define INVERTER_PIN Pin_0 // PC0 used as inverter select GPIO
#define INVERTER_GPIO GPIOC
#define INVERTER_PERIPHERAL RCC_AHB1Periph_GPIOC
#define INVERTER_USART USART1

#define MPU6500_CS_GPIO_CLK_PERIPHERAL   RCC_AHB1Periph_GPIOE
#define MPU6500_CS_GPIO       GPIOE
#define MPU6500_CS_PIN        GPIO_Pin_10
#define MPU6500_SPI_INSTANCE  SPI2

#define ACC
#define USE_ACC_MPU6500
#define USE_ACC_SPI_MPU6500
#define ACC_MPU6500_ALIGN CW270_DEG

#define GYRO
#define USE_GYRO_MPU6500
#define USE_GYRO_SPI_MPU6500
#define GYRO_MPU6500_ALIGN CW270_DEG

// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL
#define EXTI_CALLBACK_HANDLER_COUNT 2 // MPU data ready (mag disabled)

//#define MAG
//#define USE_MAG_HMC5883
//#define MAG_HMC5883_ALIGN CW90_DEG

//#define USE_MAG_NAZA
//#define MAG_NAZA_ALIGN CW180_DEG_FLIP

//#define BARO
//#define USE_BARO_MS5611

//#define PITOT
//#define USE_PITOT_MS4525
//#define MS4525_BUS I2C_DEVICE_EXT

#define INVERTER
#define BEEPER
#define LED0
#define LED1

/*
#define M25P16_CS_GPIO        GPIOB
#define M25P16_CS_PIN         GPIO_Pin_3
#define M25P16_SPI_INSTANCE   SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16
*/

#define USABLE_TIMER_CHANNEL_COUNT 12

#define USE_VCP

#define USE_USART1
#define USART1_RX_PIN Pin_7
#define USART1_TX_PIN Pin_6
#define USART1_GPIO GPIOB
#define USART1_APB2_PERIPHERALS RCC_APB2Periph_USART1
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_DMA2

#define USE_USART3
#define USART3_RX_PIN Pin_9
#define USART3_TX_PIN Pin_8
#define USART3_GPIO GPIOD
#define USART3_APB1_PERIPHERALS RCC_APB1Periph_USART3
#define USART3_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOD

#define USE_USART6
#define USART6_RX_PIN Pin_7
#define USART6_TX_PIN Pin_6
#define USART6_GPIO GPIOC
#define USART6_APB2_PERIPHERALS RCC_APB2Periph_USART6
#define USART6_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOC

#define SERIAL_PORT_COUNT 4 //VCP, USART1, USART3, USART6

#define USE_SERIAL_1WIRE
#define S1W_TX_GPIO         GPIOB
#define S1W_TX_PIN          GPIO_Pin_6
#define S1W_RX_GPIO         GPIOB
#define S1W_RX_PIN          GPIO_Pin_7

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
//#define USE_SPI_DEVICE_3

/*
#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)
*/

#define USE_ADC

#define CURRENT_METER_ADC_GPIO      GPIOA
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_5
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_2

#define VBAT_ADC_GPIO               GPIOC
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_0
#define VBAT_ADC_CHANNEL            ADC_Channel_1

#define RSSI_ADC_GPIO               GPIOB
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_1
#define RSSI_ADC_CHANNEL            ADC_Channel_12

#define SENSORS_SET (SENSOR_ACC)

#define LED_STRIP
#define LED_STRIP_TIMER TIM5

#define GPS
#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define GTUNE
#define USE_SERVOS
#define USE_CLI
