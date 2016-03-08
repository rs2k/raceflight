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

#define TARGET_BOARD_IDENTIFIER "AFF3" // AlienFlight F3.

#define USE_HARDWARE_REVISION_DETECTION
#define HW_GPIO              GPIOB
#define HW_PIN               Pin_2
#define HW_PERIPHERAL        RCC_AHBPeriph_GPIOB
    
#define CONFIG_SERIALRX_PROVIDER SERIALRX_SPEKTRUM2048
#define CONFIG_FEATURE_RX_SERIAL
#define CONFIG_RX_SERIAL_PORT 2

#define USBD_PRODUCT_STRING "AlienFlight F3"

// LED's V1
#define LED0                 PB4  // LED - PB4
#define LED1                 PB5  // LED - PB5

// LED's V2
#define LED0_A               PB8  // LED - PB8
#define LED1_A               PB9  // LED - PB9

#define BEEPER               PA5  // LED - PA5

#define USABLE_TIMER_CHANNEL_COUNT 11

//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define EXTI15_10_CALLBACK_HANDLER_COUNT 1 // MPU data ready
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define USE_EXTI

#define MPU6500_CS_PIN       PA15
#define MPU6500_SPI_INSTANCE SPI3
#define MPU9250_CS_PIN       PA15
#define MPU9250_SPI_INSTANCE SPI3

#define GYRO
#define USE_GYRO_MPU6050
#define USE_GYRO_SPI_MPU6500
#define USE_GYRO_SPI_MPU9250

#define GYRO_MPU6050_ALIGN   CW270_DEG
#define GYRO_MPU6500_ALIGN   CW270_DEG
#define GYRO_MPU9250_ALIGN   CW270_DEG

#define ACC
#define USE_ACC_MPU6050
#define USE_ACC_SPI_MPU6500
#define USE_ACC_SPI_MPU9250

#define ACC_MPU6050_ALIGN    CW270_DEG
#define ACC_MPU6500_ALIGN    CW270_DEG
#define ACC_MPU9250_ALIGN    CW270_DEG

// No baro support.
//#define BARO
//#define USE_BARO_MS5611

#define MAG
#define USE_MAG_AK8963
#define MAG_AK8963_ALIGN     CW0_DEG_FLIP

#define USE_VCP
#define USE_USART1 // Not connected - TX (PB6) RX PB7 (AF7)
#define USE_USART2 // Receiver - RX (PA3)
#define USE_USART3 // Not connected - 10/RX (PB11) 11/TX (PB10)
#define SERIAL_PORT_COUNT    4

#define UART1_TX_PIN         GPIO_Pin_6 // PB6
#define UART1_RX_PIN         GPIO_Pin_7 // PB7
#define UART1_GPIO           GPIOB
#define UART1_GPIO_AF        GPIO_AF_7
#define UART1_TX_PINSOURCE   GPIO_PinSource6
#define UART1_RX_PINSOURCE   GPIO_PinSource7

#define UART2_TX_PIN         GPIO_Pin_2 // PA2
#define UART2_RX_PIN         GPIO_Pin_3 // PA3
#define UART2_GPIO           GPIOA
#define UART2_GPIO_AF        GPIO_AF_7
#define UART2_TX_PINSOURCE   GPIO_PinSource2
#define UART2_RX_PINSOURCE   GPIO_PinSource3

#define UART3_TX_PIN         GPIO_Pin_10 // PB10 (AF7)
#define UART3_RX_PIN         GPIO_Pin_11 // PB11 (AF7)
#define UART3_GPIO_AF        GPIO_AF_7
#define UART3_GPIO           GPIOB
#define UART3_TX_PINSOURCE   GPIO_PinSource10
#define UART3_RX_PINSOURCE   GPIO_PinSource11


#define USE_I2C
#define I2C_DEVICE (I2CDEV_2) // SDA (PA10/AF4), SCL (PA9/AF4)

#define I2C2_SCL             PA9
#define I2C2_SDA             PA10

#define USE_SPI
#define USE_SPI_DEVICE_3

#define USE_ADC

#define ADC_INSTANCE         ADC2
#define ADC_DMA_CHANNEL      DMA2_Channel1
#define ADC_AHB_PERIPHERAL   RCC_AHBPeriph_DMA2

//#define BOARD_HAS_VOLTAGE_DIVIDER

#define VBAT_ADC_PIN         PA4
#define VBAT_ADC_CHANNEL     ADC_Channel_1

//#define BLACKBOX
#define SERIAL_RX
//#define GPS
#define GTUNE
//#define DISPLAY
#define USE_SERVOS
#define USE_CLI

#define SPEKTRUM_BIND
// USART2, PA3
#define BIND_PORT            GPIOA
#define BIND_PIN             Pin_3

// alternative defaults for AlienFlight F3 target
#define ALIENFLIGHT

// Hardware bind plug at PB12 (Pin 25)
#define HARDWARE_BIND_PLUG
#define BINDPLUG_PORT        GPIOB
#define BINDPLUG_PIN         Pin_12

#define TARGET_IO_PORTA      0xffff
#define TARGET_IO_PORTB      0xffff
#define TARGET_IO_PORTC      (BIT(13)|BIT(14)|BIT(15))
