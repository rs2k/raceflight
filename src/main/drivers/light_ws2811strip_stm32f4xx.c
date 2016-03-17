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

#include "platform.h"

#include "common/color.h"
#include "light_ws2811strip.h"
#include "dma.h"
#include "nvic.h"
#include "io.h"

#ifndef WS2811_PIN
#define WS2811_GPIO_AF                  GPIO_AF_TIM5
#define WS2811_PIN                      PA0 
#define WS2811_TIMER                    TIM5
#define WS2811_TIMER_APB1_PERIPHERAL    RCC_APB1Periph_TIM5
#define WS2811_DMA_HANDLER_IDENTIFER    DMA1_ST2_HANDLER
#define WS2811_DMA_STREAM               DMA1_Stream2
#define WS2811_DMA_CHANNEL              DMA_Channel_6
#define WS2811_DMA_IRQ                  DMA1_Stream2_IRQn
#endif

void ws2811DMAHandler(DMA_Stream_TypeDef *stream)
{
    if (DMA_GetFlagStatus(stream, DMA_FLAG_TCIF2)) {
        ws2811LedDataTransferInProgress = 0;
        DMA_Cmd(stream, DISABLE);
        TIM_DMACmd(WS2811_TIMER, TIM_DMA_CC1, DISABLE);
        DMA_ClearITPendingBit(stream, DMA_IT_TCIF2);
    }
}

void ws2811LedStripHardwareInit(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    uint16_t prescalerValue;

#ifdef WS2811_TIMER_APB1_PERIPHERAL
    RCC_APB1PeriphClockCmd(WS2811_TIMER_APB1_PERIPHERAL, ENABLE);
#elif WS2811_TIMER_APB2_PERIPHERAL
    RCC_APB2PeriphClockCmd(WS2811_TIMER_APB2_PERIPHERAL, ENABLE);
#endif

    /* GPIOA Configuration: TIM5 Channel 1 as alternate function push-pull */
    IOInit(IOGetByTag(IO_TAG(WS2811_PIN)), OWNER_SYSTEM, RESOURCE_OUTPUT);
    IOConfigGPIOAF(IOGetByTag(IO_TAG(WS2811_PIN)), IOCFG_AF_PP, WS2811_GPIO_AF);
    
    // Stop timer
    TIM_Cmd(WS2811_TIMER, DISABLE);

    /* Compute the prescaler value */
    prescalerValue = (uint16_t) (SystemCoreClock / 2 / 84000000) - 1;
    /* Time base configuration */
    TIM_TimeBaseStructure.TIM_Period = 104; // 800kHz
    TIM_TimeBaseStructure.TIM_Prescaler = prescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(WS2811_TIMER, &TIM_TimeBaseStructure);

    /* PWM1 Mode configuration: Channel1 */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(WS2811_TIMER, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(WS2811_TIMER, TIM_OCPreload_Enable);

    TIM_Cmd(WS2811_TIMER, ENABLE);

    dmaSetHandler(WS2811_DMA_HANDLER_IDENTIFER, ws2811DMAHandler);

    /* configure DMA */
    /* DMA1 Channel Config */
    DMA_Cmd(WS2811_DMA_STREAM, DISABLE);            // disable DMA1 stream 2
    DMA_DeInit(WS2811_DMA_STREAM);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = WS2811_DMA_CHANNEL;
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(WS2811_TIMER->CCR1);
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ledStripDMABuffer;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = WS2811_DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(WS2811_DMA_STREAM, &DMA_InitStructure);

    DMA_ITConfig(WS2811_DMA_STREAM, DMA_IT_TC, ENABLE);
    DMA_ClearITPendingBit(WS2811_DMA_STREAM, DMA_IT_TCIF2);               // clear DMA1 Channel 6 transfer complete flag

    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = WS2811_DMA_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = NVIC_PRIORITY_BASE(NVIC_PRIO_WS2811_DMA);
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = NVIC_PRIORITY_SUB(NVIC_PRIO_WS2811_DMA);
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    setStripColor(&hsv_white);
    ws2811UpdateStrip();
}

void ws2811LedStripDMAEnable(void)
{
    DMA_SetCurrDataCounter(WS2811_DMA_STREAM, WS2811_DMA_BUFFER_SIZE);  // load number of bytes to be transferred
    TIM_SetCounter(WS2811_TIMER, 0);
    DMA_Cmd(WS2811_DMA_STREAM, ENABLE);
    TIM_DMACmd(WS2811_TIMER, TIM_DMA_CC1, ENABLE);
}

