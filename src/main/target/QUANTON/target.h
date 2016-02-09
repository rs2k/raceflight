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
#define TARGET_BOARD_IDENTIFIER "QUAN"

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)

#define USE_EXTI

#define LED0 PC13
#define LED1 PC14

#define MPU6000_CS_GPIO_CLK_PERIPHERAL   RCC_AHB1Periph_GPIOC
#define MPU6000_CS_GPIO       GPIOC
#define MPU6000_CS_PIN        GPIO_Pin_4
#define MPU6000_SPI_INSTANCE  SPI1

#define ACC
#define USE_ACC_MPU6000
#define USE_ACC_SPI_MPU6000
#define ACC_MPU6000_ALIGN CW180_DEG

#define GYRO
#define USE_GYRO_MPU6000
#define USE_GYRO_SPI_MPU6000
#define GYRO_MPU6000_ALIGN CW180_DEG

//#define BARO
//#define USE_BARO_MS5611
//#define MS5611_I2C_INSTANCE I2CDEV_1

//#define M25P16_CS_GPIO        GPIOB
//#define M25P16_CS_PIN         GPIO_Pin_12
//#define M25P16_SPI_INSTANCE   SPI3

//#define USE_FLASHFS
//#define USE_FLASH_M25P16

#define USABLE_TIMER_CHANNEL_COUNT 16

// MPU6500 interrupt
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
//#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define MPU_INT_EXTI PC0

#define USE_VCP
#define VBUS_SENSING PA9

#define USE_USART1
#define USART1_RX_PIN PA3
#define USART1_TX_PIN PA2
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_DMA2

#define USE_USART3
#define USART3_RX_PIN PB11
#define USART3_TX_PIN PB10

#define USE_USART6
#define USART6_RX_PIN PC7
#define USART6_TX_PIN PC6 

#define SERIAL_PORT_COUNT 4

#define USE_ESCSERIAL
#define ESCSERIAL_TIMER_TX_HARDWARE 0

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_GPIO           GPIOC
#define SPI1_NSS_PERIPHERAL     RCC_AHBPeriph_GPIOC
#define SPI1_NSS_PIN            GPIO_Pin_4
#define SPI1_NSS_PIN_SOURCE     GPIO_PinSource4
#define SPI1_GPIO               GPIOA
#define SPI1_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOA
#define SPI1_SCK_PIN            GPIO_Pin_5
#define SPI1_SCK_PIN_SOURCE     GPIO_PinSource5
#define SPI1_MISO_PIN           GPIO_Pin_6
#define SPI1_MISO_PIN_SOURCE    GPIO_PinSource6
#define SPI1_MOSI_PIN           GPIO_Pin_7
#define SPI1_MOSI_PIN_SOURCE    GPIO_PinSource7

#define USE_SPI_DEVICE_3
#define SPI3_NSS_GPIO           GPIOB
#define SPI3_NSS_PERIPHERAL     RCC_AHBPeriph_GPIOB
#define SPI3_NSS_PIN            GPIO_Pin_3
#define SPI3_NSS_PIN_SOURCE     GPIO_PinSource3
#define SPI3_GPIO               GPIOC
#define SPI3_GPIO_PERIPHERAL    RCC_AHBPeriph_GPIOC
#define SPI3_SCK_PIN            GPIO_Pin_10
#define SPI3_SCK_PIN_SOURCE     GPIO_PinSource10
#define SPI3_MISO_PIN           GPIO_Pin_11
#define SPI3_MISO_PIN_SOURCE    GPIO_PinSource11
#define SPI3_MOSI_PIN           GPIO_Pin_12
#define SPI3_MOSI_PIN_SOURCE    GPIO_PinSource12

#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)

#define USE_ADC
#define VBAT_ADC_GPIO               GPIOC
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_12
#define VBAT_ADC_CHANNEL            ADC_Channel_13

//#define GPS
//#define BLACKBOX
#define TELEMETRY
#define SERIAL_RX
#define AUTOTUNE
#define USE_QUAD_MIXER_ONLY
#define USE_CLI

#define USE_QUATERNION

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
