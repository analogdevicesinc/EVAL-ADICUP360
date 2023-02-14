/***************************************************************************//**
 *   @file   timer.cpp
 *   @brief  Implementation of timer.cpp
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

#include <applications/message.h>
#include <vector>
#include <ctime>
#include <cstring>
#include <cassert>

using namespace std;

vector<timer> timer_v;

#define MAX_SIZE 8

int on_tick(int argc, char *argv[])
{
	for (int i = 0; i < timer_v.size(); ++i) {
		if (timer_v[i].time == 0) {
			post_message(timer_v[i].timer_app);

			if (i != timer_v.size() - 1) {
				timer_v[i] = timer_v.back();
			}

			timer_v.pop_back();

			if (timer_v.capacity() - timer_v.size() > 64) {
				timer_v.shrink_to_fit();
			}

			--i;
		} else {
			--timer_v[i].time;
		}
	}
	return 0;
}

void new_timer(timer t)
{
	if (timer_v.size() < MAX_SIZE) {
		t.time /= (1000 / CLK_TCK); //time unit is ms
		timer_v.push_back(t);
	} else {
		assert(false);
	}
}

void delete_timer(const timer &t)
{
	for (int i = 0; i < timer_v.size(); ++i) {
		if (timer_v[i].timer_app.fun == t.timer_app.fun) {
			if (timer_v[i].timer_app.argc == t.timer_app.argc) {
				bool same = true;

				for (int j = 0; j < t.timer_app.argc; ++j) { //check if all arguments same
					if (strcmp(timer_v[i].timer_app.argv[j], t.timer_app.argv[j]) != 0) {
						same = false;
						break;
					}
				}

				if (same) {
					for (int k = 0; k < timer_v[i].timer_app.argc; ++k) {
						delete[] timer_v[i].timer_app.argv[k];
					}

					delete timer_v[i].timer_app.argv;

					if (i != timer_v.size() - 1) {
						timer_v[i] = timer_v.back();
					}

					timer_v.pop_back();

					if (timer_v.capacity() - timer_v.size() > 64) {
						timer_v.shrink_to_fit();
					}

					--i; //rescan this element
				}
			}
		}
	}
}

void update_timer(const timer &t)
{
	delete_timer(t);
	new_timer(t);
}

#ifdef __cplusplus
extern "C"
{
#endif

void SysTick_Handler(void)
{
	app msg;

	msg.fun = on_tick;
	msg.argc = 0;

	ts_post_message(msg);
}

#ifdef __cplusplus
}
#endif
