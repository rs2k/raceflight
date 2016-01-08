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
#define TARGET_BOARD_IDENTIFIER "AFF4"

#define LED0_GPIO   GPIOC
#define LED0_PIN    Pin_12 // Blue LED0
#define LED0_PERIPHERAL RCC_AHB1Periph_GPIOC
#define LED1_GPIO   GPIOD
#define LED1_PIN    Pin_2  // Blue LED1
#define LED1_PERIPHERAL RCC_AHB1Periph_GPIOD

#define BEEP_GPIO   GPIOC
#define BEEP_PIN    Pin_13 // Buzzer port
#define BEEP_PERIPHERAL RCC_AHB1Periph_GPIOC
//#define BEEPER_INVERTED

#define INVERTER_GPIO GPIOC
#define INVERTER_PIN Pin_15 // PC15 used as inverter select GPIO
#define INVERTER_PERIPHERAL RCC_AHB1Periph_GPIOC
#define INVERTER_USART USART2

#define MPU9250_CS_GPIO       GPIOA
#define MPU9250_CS_PIN        GPIO_Pin_4
#define MPU9250_CS_GPIO_CLK_PERIPHERAL RCC_AHB1Periph_GPIOA
#define MPU9250_SPI_INSTANCE  SPI1

#define ACC
#define USE_ACC_MPU9250
#define USE_ACC_SPI_MPU9250
#define ACC_MPU9250_ALIGN CW270_DEG

#define GYRO
#define USE_GYRO_MPU9250
#define USE_GYRO_SPI_MPU9250
#define GYRO_MPU9250_ALIGN CW270_DEG

#define MAG
//#define USE_MAG_HMC5883
#define USE_MAG_AK8963
#define MAG_AK8963_ALIGN CW270_DEG

//#define BARO
//#define USE_BARO_MS5611
//#define USE_BARO_BMP280

#define INVERTER
#define BEEPER
#define LED0
#define LED1

#define M25P16_CS_GPIO        GPIOB
#define M25P16_CS_PIN         GPIO_Pin_12
#define M25P16_SPI_INSTANCE   SPI2

#define USE_FLASHFS
#define USE_FLASH_M25P16

#define USABLE_TIMER_CHANNEL_COUNT 13

// MPU6500 interrupt
//#define DEBUG_MPU_DATA_READY_INTERRUPT
#define USE_MPU_DATA_READY_SIGNAL
#define ENSURE_MPU_DATA_READY_IS_LOW
#define EXTI_CALLBACK_HANDLER_COUNT 1 // MPU data ready

#define USE_VCP

#define USE_USART1
#define USART1_RX_PIN Pin_10
#define USART1_TX_PIN Pin_9
#define USART1_GPIO GPIOA
#define USART1_APB2_PERIPHERALS RCC_APB2Periph_USART1
#define USART1_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_DMA2

#define USE_USART2
#define USART2_RX_PIN Pin_3
#define USART2_TX_PIN Pin_2 //inverter
#define USART2_GPIO GPIOA
#define USART2_APB1_PERIPHERALS RCC_APB1Periph_USART2
#define USART2_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOA

#define USE_USART3
#define USART3_RX_PIN Pin_11
#define USART3_TX_PIN Pin_10
#define USART3_GPIO GPIOB
#define USART3_APB1_PERIPHERALS RCC_APB1Periph_USART3
#define USART3_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOB

#define USE_USART4
#define USART4_RX_PIN Pin_10
#define USART4_TX_PIN Pin_11
#define USART4_GPIO GPIOC
#define USART4_APB1_PERIPHERALS RCC_APB1Periph_UART4
#define USART4_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOC

//#define USE_USART5
//#define USART5_RX_PIN Pin_2
//#define USART5_RXGPIO GPIOD
//#define USART5_TX_PIN Pin_12
//#define USART5_TXGPIO GPIOC
//#define USART5_APB1_PERIPHERALS RCC_APB1Periph_UART5
//#define USART5_AHB1_PERIPHERALS RCC_AHB1Periph_GPIOC

#define SERIAL_PORT_COUNT 5

//#define USE_ESCSERIAL
//#define ESCSERIAL_TIMER_TX_HARDWARE 0 // PWM 1

#define USE_SPI
#define USE_SPI_DEVICE_1
#define USE_SPI_DEVICE_2
#define USE_SPI_DEVICE_3

//#define USE_I2C
#define I2C_DEVICE (I2CDEV_1)
//#define I2C_DEVICE_EXT (I2CDEV_2)
#define I2C1_SCL_GPIO        GPIOB
#define I2C1_SCL_PIN         GPIO_Pin_6
#define I2C1_SCL_PIN_SOURCE  GPIO_PinSource6
#define I2C1_SCL_CLK_SOURCE  RCC_AHB1Periph_GPIOB
#define I2C1_SDA_GPIO        GPIOB
#define I2C1_SDA_PIN         GPIO_Pin_7
#define I2C1_SDA_PIN_SOURCE  GPIO_PinSource7
#define I2C1_SDA_CLK_SOURCE  RCC_AHB1Periph_GPIOB

#define USE_ADC
//#define BOARD_HAS_VOLTAGE_DIVIDER

#define VBAT_ADC_GPIO               GPIOC
#define VBAT_ADC_GPIO_PIN           GPIO_Pin_0
#define VBAT_ADC_CHANNEL            ADC_Channel_1

#define CURRENT_METER_ADC_GPIO      GPIOC
#define CURRENT_METER_ADC_GPIO_PIN  GPIO_Pin_1
#define CURRENT_METER_ADC_CHANNEL   ADC_Channel_0

#define RSSI_ADC_GPIO               GPIOC
#define RSSI_ADC_GPIO_PIN           GPIO_Pin_4
#define RSSI_ADC_CHANNEL            ADC_Channel_4

#define EXTERNAL1_ADC_GPIO          GPIOC
#define EXTERNAL1_ADC_GPIO_PIN      GPIO_Pin_5
#define EXTERNAL1_ADC_CHANNEL       ADC_Channel_5

// LED strip configuration using RC5 pin.
//#define LED_STRIP
//#define LED_STRIP_TIMER TIM8

//#define USE_LED_STRIP_ON_DMA1_CHANNEL3
//#define WS2811_GPIO                     GPIOB
//#define WS2811_GPIO_AHB_PERIPHERAL      RCC_AHBPeriph_GPIOB
//#define WS2811_GPIO_AF                  GPIO_AF_3
//#define WS2811_PIN                      GPIO_Pin_15 // TIM8_CH3
//#define WS2811_PIN_SOURCE               GPIO_PinSource15
//#define WS2811_TIMER                    TIM8
//#define WS2811_TIMER_APB2_PERIPHERAL    RCC_APB2Periph_TIM8
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
// USART2, PA3
#define BIND_PORT  GPIOA
#define BIND_PIN   Pin_3

// alternative defaults for AlienWii32 F4 target
#define ALIENWII32

// Hardware bind plug at PB2 (Pin 28)
#define HARDWARE_BIND_PLUG
#define BINDPLUG_PORT  GPIOB
#define BINDPLUG_PIN   Pin_2

#define USE_SERIAL_1WIRE
#define ESC_COUNT 8
#define S1W_TX_GPIO         GPIOA
#define S1W_TX_PIN          GPIO_Pin_9
#define S1W_RX_GPIO         GPIOA
#define S1W_RX_PIN          GPIO_Pin_10
