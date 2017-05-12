//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//

#include "Timer.h"
#include "cortexm/ExceptionHandlers.h"

volatile timer_ticks_t timer_delayCount;

void timer_start (void)
{
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
