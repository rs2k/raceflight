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
#include <string.h>
#include <stdint.h>

#include <platform.h>

#include "build_config.h"

#include "drivers/dma.h"

/*
 * DMA handlers for DMA resources that are shared between different features depending on run-time configuration.
 */
static dmaHandlers_t dmaHandlers;

void dmaNoOpHandler(DMA_Stream_TypeDef *stream)
{
	UNUSED(stream);
}

/*
void DMA1_Stream2_IRQHandler(void)
{
    dmaHandlers.dma1Stream2IRQHandler(DMA1_Stream2);
}

void DMA1_Stream3_IRQHandler(void)
{
    dmaHandlers.dma1Stream3IRQHandler(DMA1_Stream3);
}

void DMA1_Stream6_IRQHandler(void)
{
    dmaHandlers.dma1Stream6IRQHandler(DMA1_Stream6);
}

void DMA2_Stream1_IRQHandler(void)
{
    dmaHandlers.dma2Stream1IRQHandler(DMA2_Stream1);
}
*/

void dmaInit(void)
{
    memset(&dmaHandlers, 0, sizeof(dmaHandlers));
    dmaHandlers.dma1Stream2IRQHandler = dmaNoOpHandler;
    dmaHandlers.dma1Stream3IRQHandler = dmaNoOpHandler;
    dmaHandlers.dma1Stream6IRQHandler = dmaNoOpHandler;
    dmaHandlers.dma2Stream1IRQHandler = dmaNoOpHandler;
}

void dmaSetHandler(dmaHandlerIdentifier_e identifier, dmaCallbackHandlerFuncPtr callback)
{
    switch (identifier) {
        case DMA1_ST2_HANDLER:
            dmaHandlers.dma1Stream2IRQHandler = callback;
            break;
        case DMA1_ST3_HANDLER:
            dmaHandlers.dma1Stream3IRQHandler = callback;
            break;
        case DMA1_ST6_HANDLER:
            dmaHandlers.dma1Stream6IRQHandler = callback;
            break;
        case DMA2_ST1_HANDLER:
            dmaHandlers.dma2Stream1IRQHandler = callback;
            break;
    }
}
