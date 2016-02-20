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

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)
#define CONFIG_SERIALRX_PROVIDER SERIALRX_SBUS
//#define CONFIG_BLACKBOX_DEVICE BLACKBOX_DEVICE_FLASH
#define CONFIG_FEATURE_RX_SERIAL
#define CONFIG_FEATURE_ONESHOT125
#define CONFIG_MSP_PORT 1
#define CONFIG_RX_SERIAL_PORT 4

#define USBD_PRODUCT_STRING "VR Brain Core"

#define LED0 PD14 // Blue LED
#define LED1 PD15 // Red LED
#define BEEPER PA0

#define MPU9250_CS_PIN        PE10
#define MPU9250_SPI_INSTANCE  SPI2

#define ACC
#define USE_ACC_MPU9250
#define USE_ACC_SPI_MPU9250
#define ACC_MPU9250_ALIGN CW270_DEG

#define GYRO
#define USE_GYRO_MPU9250
#define USE_GYRO_SPI_MPU9250
#define GYRO_MPU9250_ALIGN CW270_DEG

// MPU6000 interrupts
#define USE_MPU_DATA_READY_SIGNAL
#define EXTI_CALLBACK_HANDLER_COUNT 2 // MPU data ready (mag disabled)
#define MPU_INT_EXTI PD10
#define USE_EXTI

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

/*
#define M25P16_CS_PIN         PB3
#define M25P16_SPI_INSTANCE   SPI3

#define USE_FLASHFS
#define USE_FLASH_M25P16
*/

#define USABLE_TIMER_CHANNEL_COUNT 12

#define USE_VCP
#define VBUS_SENSING_PIN PA9

#define USE_USART1
#define USART1_RX_PIN PB7
#define USART1_TX_PIN PB6
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_DMA2

#define USE_USART2
#define USART2_RX_PIN PD6
#define USART2_TX_PIN PD5

#define USE_USART3
#define USART3_RX_PIN PD9
#define USART3_TX_PIN PD8

#define USE_USART6
#define USART6_RX_PIN PC7
#define USART6_TX_PIN PC6

#define INVERTER PD7
#define INVERTER_USART USART6

#define SERIAL_PORT_COUNT 5 //VCP, USART1, USART2, USART3, USART6

#define USE_SERIAL_1WIRE
#define S1W_TX_GPIO         GPIOB
#define S1W_TX_PIN          GPIO_Pin_6
#define S1W_RX_GPIO         GPIOB
#define S1W_RX_PIN          GPIO_Pin_7

#define USE_SPI

#define USE_SPI_DEVICE_1

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PE10
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

//#define USE_SPI_DEVICE_3

/*
#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)
*/

#define USE_ADC

#define CURRENT_METER_ADC_PIN       PA5
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_2

#define VBAT_ADC_PIN                PC0
#define VBAT_ADC_CHANNEL            ADC_Channel_1

#define RSSI_ADC_GPIO_PIN           PB1
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

#define USE_QUATERNION

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
