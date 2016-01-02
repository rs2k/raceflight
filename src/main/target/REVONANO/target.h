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
#define TARGET_BOARD_IDENTIFIER "REVN"

#define LED0_GPIO   GPIOC
#define LED0_PIN    Pin_14 // Blue LEDs - PC14
#define LED0_PERIPHERAL RCC_AHB1Periph_GPIOC
#define LED1_GPIO   GPIOC
#define LED1_PIN    Pin_13  // Orange LEDs - PC13
#define LED1_PERIPHERAL RCC_AHB1Periph_GPIOC

#define BEEP_GPIO GPIOC
#define BEEP_PIN Pin_13 // Orange LEDs - PC13
#define BEEP_PERIPHERAL RCC_AHB1Periph_GPIOC

#define INVERTER_PIN Pin_15 // PC15 used as inverter select GPIO
#define INVERTER_GPIO GPIOC
#define INVERTER_PERIPHERAL RCC_AHB1Periph_GPIOC
#define INVERTER_USART USART2 //Sbus on USART 2 of nano.

#define MPU9250_CS_GPIO_CLK_PERIPHERAL   RCC_AHB1Periph_GPIOB
#define MPU9250_CS_GPIO       GPIOB
#define MPU9250_CS_PIN        GPIO_Pin_12
#define MPU9250_SPI_INSTANCE  SPI2

#define ACC
#define USE_ACC_MPU9250
#define USE_ACC_SPI_MPU9250
#define ACC_MPU9250_ALIGN CW270_DEG

#define GYRO
#define USE_GYRO_MPU9250
#define USE_GYRO_SPI_MPU9250
#define GYRO_MPU9250_ALIGN CW270_DEG

//#define MAG
//#define USE_MAG_HMC5883

#define BARO
#define USE_BARO_MS5611

#define INVERTER
#define BEEPER
#define LED0
#define LED1

// MPU9250 interrupts
#define USE_MPU_DATA_READY_SIGNAL
//#define ENSURE_MPU_DATA_READY_IS_LOW
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready (mag disabled)


#define USABLE_TIMER_CHANNEL_COUNT 13

#define USE_VCP

#define USE_USART1
#define USART1_RX_PIN Pin_7
#define USART1_TX_PIN Pin_6
#define USART1_GPIO GPIOB
#define USART1_APB2_PERIPHERALS RCC_APB2Periph_USART1
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOB

#define USE_USART2
#define USART2_RX_PIN Pin_3
#define USART2_TX_PIN Pin_2
#define USART2_GPIO GPIOA
#define USART2_APB1_PERIPHERALS RCC_APB1Periph_USART2
#define USART2_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOA

#define SERIAL_PORT_COUNT 3 //VCP, USART1, USART2

#define USE_SPI
//#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
//#define USE_SPI_DEVICE_3

#define USE_I2C
#define I2C_DEVICE (I2CDEV_3)

#define USE_ADC

//FLEXI-IO	6
#define CURRENT_METER_ADC_GPIO      GPIOA
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_7
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_7

//FLEXI-IO	7
#define VBAT_ADC_GPIO               GPIOA
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_6
#define VBAT_ADC_CHANNEL            ADC_Channel_6

//FLEXI-IO	8
#define RSSI_ADC_GPIO               GPIOA
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_5
#define RSSI_ADC_CHANNEL            ADC_Channel_5


//#define SENSORS_SET (SENSOR_ACC|SENSOR_MAG)

//#define LED_STRIP
//#define LED_STRIP_TIMER TIM5

//#define GPS
#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define GTUNE
#define USE_SERVOS
#define USE_CLI

#define USE_SERIAL_1WIRE

#define S1W_TX_GPIO         GPIOB
#define S1W_TX_PIN          GPIO_Pin_6

#define S1W_RX_GPIO         GPIOB
#define S1W_RX_PIN          GPIO_Pin_7
