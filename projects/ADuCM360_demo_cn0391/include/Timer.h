//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//
// Portions Copyright (c) 2017 Analog Devices, Inc.
//

#ifndef TIMER_H_
#define TIMER_H_

#include <adi_processor.h>

extern uint32_t SystemCoreClock;  /*!< System Clock Frequency (Core Clock)*/

// ----------------------------------------------------------------------------

class Timer
{
public:
  typedef uint32_t ticks_t;
  static constexpr ticks_t FREQUENCY_HZ = 1000u;

private:
  static volatile ticks_t ms_delayCount;

public:
  // Default constructor
  Timer() = default;

  inline void
  start(void)
  {
	SystemCoreClockUpdate();
    // Use SysTick as reference for the delay loops.
    SysTick_Config(SystemCoreClock / FREQUENCY_HZ);
  }

  static void
  sleep(ticks_t ticks);

  inline static void
  tick(void)
  {
    // Decrement to zero the counter used by the delay routine.
    if (ms_delayCount != 0u)
      {
        --ms_delayCount;
      }
  }
};

extern Timer timer;
// ----------------------------------------------------------------------------

#endif // TIMER_H_
