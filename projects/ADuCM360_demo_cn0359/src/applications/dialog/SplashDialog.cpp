/***************************************************************************//**
 *   @file   SplashDialog.cpp
 *   @brief  Implementation of SplashDialog.cpp
 *   @author
 *******************************************************************************
 * Copyright 2022(c) Analog Devices, Inc.
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

#include <applications/dialog/HomeDialog.h>
#include <applications/dialog/SplashDialog.h>
#include <hal/devices.h>
#include <cstdio>
#include <hal/timer.h>
#include <hal/drivers/flash.h>
#include <applications/dialog/ContrastDialog.h>
#include <hal/drivers/buzzer.h>

CSplashDialog SplashDialog;

static int on_time(int argc, char *argv[])
{
	beep(200);
	p_dialog = &HomeDialog;
	p_dialog->OnCreat();
}

void CSplashDialog::OnButtonDown(void)
{
	timer t;
	t.time = 100;
	t.timer_app.argc = 0;
	t.timer_app.fun = on_time;

	delete_timer(t);

	beep(100, 100, 100);
	p_dialog = &ContrastDialog;
	p_dialog->OnCreat();
}

void CSplashDialog::OnCreat(void)
{
	rewind(p_lcd);

	fputs(" ANALOG DEVICES ", p_lcd);
	fputs("AHEAD OF WHAT'S ", p_lcd);
	fputs("    POSSIBLE    ", p_lcd);
	fputs("EVAL-CN0359-EB1Z", p_lcd);

	fflush(p_lcd);

	timer t;
	t.time = 3000;
	t.timer_app.argc = 0;
	t.timer_app.fun = on_time;
	t.timer_app.argv = new char*;

	new_timer(t);
}

