/*!
 *****************************************************************************
 * @file:    Timer.cpp
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

/***************************** Include Files **********************************/

#include "Timer.h"

#include <cassert>

#include <cstdio>
#include <cstring>

#include <ctime>
#include <time.h>
#include <sys/times.h>

#include "Cmd.h"


#define MAX_TIMER_SIZE 8

struct STimer {
   unsigned long time = 0;
   void (*pFun)(void);
public:
   STimer(void) {
      pFun = nullptr;
   }
} Timer[MAX_TIMER_SIZE];


void Timer_add(const unsigned long ms, void (*pF)(void))
{
   for (auto & a : Timer) {
      if (a.pFun == nullptr) {
         a.time = (((ms * TICK_CLK) / 1000) + 1);
         a.pFun = pF;
         break;
      }

      assert((&a) != (&Timer[MAX_TIMER_SIZE - 1]));
   }
}

void Timer_del(void (*pF)(void))
{
   __disable_irq();

   for (auto & a : Timer) {
      if (a.pFun == pF) {
         a.pFun = nullptr;
      }
   }

   __enable_irq();
}

void Timer_start(void)
{
	SystemCoreClockUpdate();
    SysTick_Config(SystemCoreClock / TICK_CLK);
}

#ifdef __cplusplus
extern "C"
{
#endif

static volatile long long timer;

int _times(struct tms *buf)
{
   buf->tms_stime = timer;
   buf->tms_utime = 0;
   buf->tms_cstime = 0;
   buf->tms_cutime = 0;

   return timer;
}

void SysTick_Handler(void)
{


   ++timer;

   for (auto & a : Timer) {
      if (a.pFun != nullptr) {
         if ((--a.time) == 0) {
            Cmd_SetMsg(a.pFun);
            a.pFun = nullptr;
         }
      }
   }
}

#ifdef __cplusplus
}
#endif
