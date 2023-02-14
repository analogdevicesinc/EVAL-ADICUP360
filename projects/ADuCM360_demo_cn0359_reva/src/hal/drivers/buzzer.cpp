/***************************************************************************//**
 *   @file   buzzer.cpp
 *   @brief  Implementation of buzzer.cpp
 *   @author
 *******************************************************************************
 * Copyright 2023(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/
#include <ADuCM360.h>
#include <hal/timer.h>
#include <list>

#include <cstdarg>

using namespace std;

list<unsigned int> beep_l;

void buzzer_open(void)
{
	pADI_GP2->GPCON = (pADI_GP2->GPCON & ~GP2CON_CON2_MSK) | GP2CON_CON2_GPIO;

	GP2OCE_OCE2_BBA = false;

	GP2SET_SET2_BBA = true;

	GP2OEN_OEN2_BBA = true;

	GP2PUL_PUL2_BBA = true;
}

int static beep_loop(int argc, char *argv[])
{
	timer t;

	if (!beep_l.empty())
	{
		t.time = beep_l.front();
		beep_l.pop_front();

		t.timer_app.fun = beep_loop;
		t.timer_app.argc = 0;
		t.timer_app.argv = new char*;

		GP2TGL_TGL2_BBA = 0x1;

		update_timer(t);
	}
	else
	{
		t.timer_app.fun = beep_loop;
		t.timer_app.argc = 0;

		delete_timer(t);
		GP2SET_SET2_BBA = 0x1; //stop
	}
}

void _beep(int time, ...)
{
	GP2SET_SET2_BBA = 0x1; //stop

	beep_l.clear();

	timer t;
	t.timer_app.fun = beep_loop;
	t.timer_app.argc = 0;

	delete_timer(t);

	va_list ap;
	va_start(ap, time);

	for (int i = time; i > 0; i = va_arg(ap, int))
	{
		beep_l.push_back(i);
	}

	va_end(ap);

	beep_loop(0, nullptr); //start
}
