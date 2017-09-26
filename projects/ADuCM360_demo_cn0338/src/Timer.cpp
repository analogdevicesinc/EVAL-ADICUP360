//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//
// Portions Copyright (c) 2017 Analog Devices, Inc.
//

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
