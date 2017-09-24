/*!
 *****************************************************************************
 * @file:    Timer.c
 * @brief:   Timer functions
 * @version: $Revision$
 * @date:    $Date$
 *-----------------------------------------------------------------------------
 *
Copyright (c) 2015-2017 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
  - Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
  - Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
  - Modified versions of the software must be conspicuously marked as such.
  - This software is licensed solely and exclusively for use with processors
    manufactured by or for Analog Devices, Inc.
  - This software may not be combined or merged with other code in any manner
    that would cause the software to become subject to terms and conditions
    which differ from those listed here.
  - Neither the name of Analog Devices, Inc. nor the names of its
    contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.
  - The use of this software may or may not infringe the patent rights of one
    or more patent holders.  This license does not release you from the
    requirement that you obtain separate licenses from these patent holders
    to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

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
