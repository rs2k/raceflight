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

#if defined(STM32F40_41xxx) || defined (STM32F411xE)

#define SPI_SLOW_CLOCK      128 //00.65625 MHz
#define SPI_STANDARD_CLOCK    8 //11.50000 MHz
#define SPI_FAST_CLOCK        4 //21.00000 MHz
#define SPI_ULTRAFAST_CLOCK   2 //42.00000 MHz

#else

#define SPI_SLOW_CLOCK       128 //00.56250 MHz
#define SPI_STANDARD_CLOCK     4 //09.00000 MHz
#define SPI_FAST_CLOCK         2 //18.00000 MHz
#define SPI_ULTRAFAST_CLOCK    2 //18.00000 MHz

#endif

bool spiInit(SPI_TypeDef *instance);
void spiSetDivisor(SPI_TypeDef *instance, uint16_t divisor);
uint8_t spiTransferByte(SPI_TypeDef *instance, uint8_t in);

bool spiTransfer(SPI_TypeDef *instance, uint8_t *out, const uint8_t *in, int len);

uint16_t spiGetErrorCounter(SPI_TypeDef *instance);
void spiResetErrorCounter(SPI_TypeDef *instance);
