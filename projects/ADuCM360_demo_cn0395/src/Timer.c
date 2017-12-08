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
