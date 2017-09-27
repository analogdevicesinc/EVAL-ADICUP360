/*
* Copyright (c) 2014 Liviu Ionescu.
* Copyright (c) 2017 Analog Devices, Inc.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
* THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
* OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*/


/***************************** Include Files **********************************/

#include "Timer.h"
#include "cortexm/ExceptionHandlers.h"

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
