//
// This file is part of the GNU ARM Eclipse Plug-ins project.
// Copyright (c) 2014 Liviu Ionescu.
//
// Portions Copyright (c) 2017 Analog Devices, Inc.
//

#ifndef TIMER_H_
#define TIMER_H_

#include "ADuCM360.h"

#pragma once

#define TICK_CLK 100ull

extern void Timer_add(const unsigned long ms, void (*pF)(void));
extern void Timer_del(void (*pF)(void));
extern void Timer_start(void);


#endif // TIMER_H_
