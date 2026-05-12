/*
 * pid_support.c
 *
 *  Created on: Sep 16, 2020
 *      Author: raysw
 */

#include "stm32f4xx_hal.h"
#include "pid_support.h"

pid_ticks_t pid_ticks_per_second(void)
{
    return (1000U / HAL_GetTickFreq());
}

pid_ticks_t pid_ticks_get()
{
    return HAL_GetTick();
}
