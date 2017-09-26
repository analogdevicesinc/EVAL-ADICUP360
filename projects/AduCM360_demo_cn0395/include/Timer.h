//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//
// Portions Copyright (c) 2017 Analog Devices, Inc.
//

#ifndef TIMER_H_
#define TIMER_H_

#include <adi_processor.h>

// ----------------------------------------------------------------------------

#define TIMER_FREQUENCY_HZ (1000u)

typedef uint32_t timer_ticks_t;

extern volatile timer_ticks_t timer_delayCount;

extern void
timer_start (void);

extern void
timer_sleep (timer_ticks_t ticks);

void
timer_start_us (void);

void
timer_tick_us (void);

void timer_sleep_5uS(timer_ticks_t ticks);

// ----------------------------------------------------------------------------

#endif // TIMER_H_
