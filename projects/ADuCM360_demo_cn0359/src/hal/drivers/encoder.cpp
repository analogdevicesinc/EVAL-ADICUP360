/***************************************************************************//**
 *   @file   encoder.cpp
 *   @brief  Implementation of encoder.cpp
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

#include <applications/dialog/Dialog.h>
#include <ADuCM360.h>
#include <hal/timer.h>
#include <applications/message.h>

void encoder_open(void)
{
	pADI_GP1->GPCON = (pADI_GP1->GPCON & ~GP1CON_CON1_MSK) | GP1CON_CON1_GPIOIRQ4;
	pADI_GP1->GPCON = (pADI_GP1->GPCON & ~GP1CON_CON5_MSK) | GP1CON_CON5_GPIOIRQ5;
	pADI_GP1->GPCON = (pADI_GP1->GPCON & ~GP1CON_CON7_MSK) | GP1CON_CON7_GPIOIRQ7;

	GP1OCE_OCE1_BBA = false;
	GP1OCE_OCE5_BBA = false;
	GP1OCE_OCE7_BBA = false;

	GP1PUL_PUL1_BBA = true;
	GP1PUL_PUL5_BBA = true;
	GP1PUL_PUL7_BBA = true;

	GP1OEN_OEN1_BBA = false;
	GP1OEN_OEN5_BBA = false;
	GP1OEN_OEN7_BBA = false;

	pADI_INTERRUPT->EI1CFG = (pADI_INTERRUPT->EI1CFG & ~EI1CFG_IRQ4MDE_MSK) |
				 EI1CFG_IRQ4MDE_RISEORFALL;
	pADI_INTERRUPT->EI1CFG = (pADI_INTERRUPT->EI1CFG & ~EI1CFG_IRQ5MDE_MSK) |
				 EI1CFG_IRQ5MDE_RISEORFALL;
	pADI_INTERRUPT->EI1CFG = (pADI_INTERRUPT->EI1CFG & ~EI1CFG_IRQ7MDE_MSK) |
				 EI1CFG_IRQ7MDE_RISEORFALL;

	EI1CFG_IRQ4EN_BBA = true;
	EI1CFG_IRQ5EN_BBA = true;
	EI1CFG_IRQ7EN_BBA = true;

	NVIC_SetPriority(EINT4_IRQn, NVIC_EncodePriority(6, 1, 2));
	NVIC_SetPriority(EINT5_IRQn, NVIC_EncodePriority(6, 1, 2));
	NVIC_SetPriority(EINT7_IRQn, NVIC_EncodePriority(6, 1, 2));

	NVIC_EnableIRQ(EINT4_IRQn);
	NVIC_EnableIRQ(EINT5_IRQn);
	NVIC_EnableIRQ(EINT7_IRQn);
}

static int debounce(int argc, char *argv[])
{
	app msg;
	msg.argc = 0;
	msg.argv = new char*;

	if (GP1IN_IN7_BBA) {
		msg.fun = on_button_up;
	} else {

		msg.fun = on_button_down;
	}

	post_message(msg);

	EI1CFG_IRQ7EN_BBA = true;
}

static int on_int7(int argc, char *argv[])
{
	timer t;
	t.time = 50;
	t.timer_app.argc = 0;
	t.timer_app.fun = debounce;
	t.timer_app.argv = new char*;

	new_timer(t);
}

#ifdef __cplusplus
extern "C"
{
#endif
static int in1, in5, count;

void Ext_Int4_Handler(void) //encoder
{
	EI1CFG_IRQ4EN_BBA = false;

	if (count < 3) {
		++count;
	} else {
		count = 0;

		app msg;
		msg.argc = 0;

		if (GP1IN_IN5_BBA == in1) {
			msg.fun = on_cw;
		} else {
			msg.fun = on_ccw;
		}

		ts_post_message(msg);
	}

	in5 = GP1IN_IN5_BBA;

	EICLR_IRQ4_BBA = true;
	EI1CFG_IRQ5EN_BBA = true;
	__DSB();
}

void Ext_Int5_Handler(void) //encoder
{
	EI1CFG_IRQ5EN_BBA = false;

	if (count < 3) {
		++count;
	} else {
		count = 0;

		app msg;
		msg.argc = 0;

		if (GP1IN_IN1_BBA == in5) {
			msg.fun = on_ccw;
		} else {
			msg.fun = on_cw;
		}

		ts_post_message(msg);
	}

	in1 = GP1IN_IN1_BBA;

	EICLR_IRQ5_BBA = true;
	EI1CFG_IRQ4EN_BBA = true;
	__DSB();
}

void Ext_Int7_Handler(void) //encoder button
{
	EI1CFG_IRQ7EN_BBA = false;

	app msg;
	msg.argc = 0;
	msg.fun = on_int7;

	ts_post_message(msg);

	EICLR_IRQ7_BBA = true;
	__DSB();
}
#ifdef __cplusplus
}
#endif
