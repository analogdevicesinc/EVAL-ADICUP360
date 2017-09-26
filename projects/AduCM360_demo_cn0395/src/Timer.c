//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//
// Portions Copyright (c) 2017 Analog Devices, Inc.
//

#include "Timer.h"
#include "AD7988.h"
#include "GptLib.h"

// ----------------------------------------------------------------------------

// Forward declarations.

void
timer_tick (void);

// ----------------------------------------------------------------------------

volatile timer_ticks_t timer_delayCount;
volatile timer_ticks_t timer_delayCount_us;


// ----------------------------------------------------------------------------

void
timer_start (void)
{
   // Use general purpose timer for 1ms delays
   GptLd(pADI_TM0,62); // 1ms
   GptCfg(pADI_TM0,TCON_CLK_UCLK,TCON_PRE_DIV256,TCON_MOD|TCON_RLD|TCON_ENABLE);  // T0 config, Uclk/256,

   NVIC_EnableIRQ(TIMER0_IRQn);
}

void
timer_start_us (void)
{
	SystemCoreClockUpdate();
    // Use SysTick for 5uS delays
    SysTick_Config (SystemCoreClock / 180000);
}

void
timer_sleep (timer_ticks_t ticks)
{
   timer_delayCount = ticks;

   // Busy wait until the SysTick decrements the counter to zero.
   while (timer_delayCount != 0u)
      ;
}

void timer_sleep_5uS(timer_ticks_t ticks)
{
   timer_delayCount_us = ticks;

   // Busy wait until the SysTick decrements the counter to zero.
   while (timer_delayCount_us != 0u)
     ;
}


void
timer_tick (void)
{
   // Decrement to zero the counter used by the delay routine.
   if (timer_delayCount != 0u) {
      --timer_delayCount;
   }
}

void
timer_tick_us (void)
{
  // Decrement to zero the counter used by the delay routine.
  if (timer_delayCount_us != 0u)
    {
      --timer_delayCount_us;
    }
}

// ----- SysTick_Handler() ----------------------------------------------------

void
SysTick_Handler (void)
{
   timer_tick_us();
}

void GP_Tmr0_Int_Handler(void)
{
   GptClrInt(pADI_TM0,TSTA_TMOUT);
   timer_tick ();
}



// ----------------------------------------------------------------------------
