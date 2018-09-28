/*!
 *****************************************************************************
 * @file:    Timer.h
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

#ifndef TIMER_H_
#define TIMER_H_

#include <adi_processor.h>

// ----------------------------------------------------------------------------

#define TIMER_FREQUENCY_HZ (1000u)

#define STEP 1
#define CASE1 1*STEP
#define CASE2 2*STEP
#define CASE3 3*STEP
#define CASE4 4*STEP
#define CASE5 5*STEP
#define CASE6 6*STEP

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
