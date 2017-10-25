/*!
 *****************************************************************************
 * @file:    Timer.c
 * @brief:
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2014 Liviu Ionescu.

Portions Copyright (c) 2017 Analog Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a
copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.
*/

#include "Timer.h"

volatile timer_ticks_t timer_delayCount;

void timer_start (void)
{
	SystemCoreClockUpdate();
    SysTick_Config (SystemCoreClock / TIMER_FREQUENCY_HZ);
}

void timer_sleep (timer_ticks_t ticks)
{
   timer_delayCount = ticks;

   while (timer_delayCount != 0u);
}

void timer_tick (void)
{
   if (timer_delayCount != 0u) {
      --timer_delayCount;
   }
}

void SysTick_Handler (void)
{
#if defined(USE_HAL_DRIVER)
   HAL_IncTick();
#endif
   timer_tick ();
}
