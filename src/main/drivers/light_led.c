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
#include <stdlib.h>

#include "platform.h"

#include "common/utils.h"

#include "io.h"
#include "io_impl.h"

#include "light_led.h"

static const IO_t leds[] = {
#ifdef LED0
    DEFIO_IO(LED0),
#else
    DEFIO_IO(NONE),				  
#endif 
#ifdef LED1
    DEFIO_IO(LED1),
#else
    DEFIO_IO(NONE),				  
#endif 
#ifdef LED2
    DEFIO_IO(LED2),
#else
    DEFIO_IO(NONE),				  
#endif 
};

uint8_t ledPolarity = 0
#ifdef LED0_INVERTED
    | BIT(0)
#endif
#ifdef LED1_INVERTED
    | BIT(1)
#endif
#ifdef LED2_INVERTED
    | BIT(2)
#endif
    ;

void ledInit(void)
{
	uint32_t i;

	LED0_OFF;
	LED1_OFF;
	LED2_OFF;

	for (i = 0; i < ARRAYLEN(leds); i++) {
		if (leds[i]) {
			IOInit(leds[i], OWNER_SYSTEM, RESOURCE_OUTPUT);
			IOConfigGPIO(leds[i], IOCFG_OUT_PP);
		}
	}

    LED0_OFF;
    LED1_OFF;
    LED2_OFF;
}

void ledToggle(int led)
{
	IOToggle(leds[led]);
}

void ledSet(int led, bool on)
{
	bool inverted = (1 << led) & ledPolarity;
	IOWrite(leds[led], on ? inverted : !inverted);
}
