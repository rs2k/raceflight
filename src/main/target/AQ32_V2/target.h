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
#define TARGET_BOARD_IDENTIFIER "AQ32"

#define CONFIG_START_FLASH_ADDRESS (0x08080000) //0x08080000 to 0x080A0000 (FLASH_Sector_8)
#define CONFIG_SERIALRX_PROVIDER SERIALRX_SBUS
#define CONFIG_BLACKBOX_DEVICE BLACKBOX_DEVICE_SDCARD
#define CONFIG_FEATURE_RX_SERIAL
#define CONFIG_RX_SERIAL_PORT 2
#define CONFIG_FEATURE_ONESHOT125
#define CONFIG_MSP_PORT 1

#define USBD_PRODUCT_STRING "AeroQuad32 V2"

#define LED0 PE5
#define LED0_INVERTED
#define LED1 PE6
#define LED1_INVERTED
#define LED2 PD7 // LED2 external with transistor driver

#define BEEPER PE0 // LED1 external with transistor driver

#define INVERTER PB12
#define INVERTER_USART USART3

#define MPU6000_CS_PIN        PB8
#define MPU6000_SPI_INSTANCE  SPI3

#define ACC
#define USE_ACC_MPU6000
#define USE_ACC_SPI_MPU6000
//#define ACC_MPU6000_ALIGN CW0_DEG

#define GYRO
#define USE_GYRO_MPU6000
#define USE_GYRO_SPI_MPU6000
//#define GYRO_MPU6000_ALIGN CW0_DEG

#define MAG
#define USE_MAG_HMC5883
#define MAG_HMC5883_ALIGN CW90_DEG

#define BARO
#define USE_BARO_MS5611
#define MS5611_ADDR           0x76

#define USE_SDCARD

//#define SDCARD_DETECT_INVERTED

#define SDCARD_DETECT_PIN                   PC1
//#define SDCARD_DETECT_EXTI_LINE             EXTI_Line1
//#define SDCARD_DETECT_EXTI_PIN_SOURCE       EXTI_PinSource1
//#define SDCARD_DETECT_EXTI_PORT_SOURCE      EXTI_PortSourceGPIOC
//#define SDCARD_DETECT_EXTI_IRQn             EXTI2_IRQn

#define SDCARD_SPI_INSTANCE                 SPI1
#define SDCARD_SPI_CS_PIN                   PE10

// SPI2 is on the APB1 bus whose clock runs at 84MHz. Divide to under 400kHz for init:
#define SDCARD_SPI_INITIALIZATION_CLOCK_DIVIDER 256 // 328kHz
// Divide to under 25MHz for normal operation:
#define SDCARD_SPI_FULL_SPEED_CLOCK_DIVIDER     4 // 21MHz

#define SDCARD_DMA_CHANNEL_TX               DMA2_Stream5
#define SDCARD_DMA_CHANNEL_TX_COMPLETE_FLAG DMA_FLAG_TCIF5
#define SDCARD_DMA_CLK                      RCC_AHB1Periph_DMA2
#define SDCARD_DMA_CHANNEL                  DMA_Channel_3

// Performance logging for SD card operations:
// #define AFATFS_USE_INTROSPECTIVE_LOGGING

#define USABLE_TIMER_CHANNEL_COUNT 20

#define USE_EXTI
#define EXTI_CALLBACK_HANDLER_COUNT 2 // MPU data ready
// MPU6000 interrupt
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define MPU_INT_EXTI PE4
// HMC5883 interrupt
#define USE_MAG_DATA_READY_SIGNAL
#define ENSURE_MAG_DATA_READY_IS_HIGH

#define USE_VCP

#define USE_USART1
#define USART1_RX_PIN PA10
#define USART1_TX_PIN PA9
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_DMA2

#define USE_USART2
#define USART2_RX_PIN PD6
#define USART2_TX_PIN PD5

#define USE_USART3
#define USART3_RX_PIN PD9 //inverter
#define USART3_TX_PIN PD8 //inverter

#define SERIAL_PORT_COUNT 4

#define USE_SPI

#define USE_SPI_DEVICE_1
#define SPI1_NSS_PIN            PE10
#define SPI1_SCK_PIN            PA5
#define SPI1_MISO_PIN           PA6
#define SPI1_MOSI_PIN           PA7

#define USE_SPI_DEVICE_2
#define SPI2_NSS_PIN            PA3
#define SPI2_SCK_PIN            PB13
#define SPI2_MISO_PIN           PB14
#define SPI2_MOSI_PIN           PB15

#define USE_SPI_DEVICE_3
#define SPI3_NSS_PIN            PB8
#define SPI3_SCK_PIN            PC10
#define SPI3_MISO_PIN           PC11
#define SPI3_MOSI_PIN           PC12

#define USE_I2C

#define I2C_DEVICE (I2CDEV_1)
#define I2C1_SCL    PB6
#define I2C1_SDA    PB7

#define I2C_DEVICE_EXT (I2CDEV_2)
#define I2C2_SCL    PB10
#define I2C2_SDA    PB11

#define USE_ADC

#define VBAT_ADC_PIN                PC0
#define VBAT_ADC_CHANNEL            ADC_Channel_10

#define CURRENT_METER_ADC_PIN       PC4
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_14

#define RSSI_ADC_PIN                PB1
#define RSSI_ADC_CHANNEL            ADC_Channel_9

#define EXTERNAL1_ADC_GPIO_PIN      PC5
#define EXTERNAL1_ADC_CHANNEL       ADC_Channel_15

// LED strip configuration using SVR3 pin.
//#define LED_STRIP
//#define LED_STRIP_TIMER TIM5

//#define USE_LED_STRIP_ON_DMA1_CHANNEL3
//#define WS2811_GPIO                     GPIOA
//#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOA
//#define WS2811_GPIO_AF                  GPIO_AF_3
//#define WS2811_PIN                      GPIO_Pin_0 // TIM5_CH1
//#define WS2811_PIN_SOURCE               GPIO_PinSource0
//#define WS2811_TIMER                    TIM5
//#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM5
//#define WS2811_DMA_CHANNEL              DMA1_Channel3
//#define WS2811_IRQ                      DMA1_Channel3_IRQn

#define BLACKBOX
//#define DISPLAY
#define GPS
#define GTUNE
#define SERIAL_RX
#define TELEMETRY
#define USE_SERVOS
#define USE_CLI

#define SPEKTRUM_BIND
// USART2, PD6
#define BIND_PORT  GPIOD
#define BIND_PIN   Pin_6

#define USE_SERIAL_1WIRE
#define ESC_COUNT 8
#define S1W_TX_GPIO         GPIOA
#define S1W_TX_PIN          GPIO_Pin_9
#define S1W_RX_GPIO         GPIOA
#define S1W_RX_PIN          GPIO_Pin_10

#define USE_QUATERNION

#define TARGET_IO_PORTA 0xffff
#define TARGET_IO_PORTB 0xffff
#define TARGET_IO_PORTC 0xffff
#define TARGET_IO_PORTD 0xffff
#define TARGET_IO_PORTE 0xffff
