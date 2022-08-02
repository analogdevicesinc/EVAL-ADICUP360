/***************************************************************************//**
 *   @file   devices.cpp
 *   @brief  Implementation of devices.cpp
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

#include <cstdio>
#include <cstring>
#include <ADuCM360.h>
#include <hal/drivers/uart.h>
#include <hal/drivers/pwm.h>
#include <hal/drivers/ad8253.h>
#include <hal/drivers/adc.h>
#include <hal/drivers/encoder.h>
#include <hal/drivers/buzzer.h>
#include <hal/drivers/flash.h>
#include <hal/drivers/dac.h>

FILE *p_lcd, *p_uart, *p_flash, *p_dac, *p_adc, *p_pwm, *p_ad8253;

void initial_devices(void)
{
	setvbuf(stdout, nullptr, _IOLBF, 64);
	setvbuf(stdin, nullptr, _IOFBF, 16);
	setvbuf(stderr, nullptr, _IONBF, 0);

	NVIC_SetPriorityGrouping(6); //2 groups, each group have 4 subpriorities

	buzzer_open();

	p_flash = fopen("flash", "rb+");
	setvbuf(p_flash, nullptr, _IOFBF, sizeof(flash_file));

	flash_file setting;
	rewind(p_flash);
	fread(&setting, sizeof(flash_file), 1, p_flash);

	if (setting.baud_rate <= configure_file.max_baud_rate
	    && setting.baud_rate >= configure_file.min_baud_rate
	    && setting.cell_const <= configure_file.max_cell_const
	    && setting.cell_const >= configure_file.min_cell_const
	    && setting.frequency <= configure_file.max_freq
	    && setting.frequency >= configure_file.min_freq
	    && setting.temp_coef <= configure_file.max_temp_coef
	    && setting.temp_coef >= configure_file.min_temp_coef
	    && setting.voltage <= configure_file.max_voltage
	    && setting.voltage >= configure_file.min_voltage
	    && setting.setup <= configure_file.max_setup
	    && setting.setup >= configure_file.min_setup
	    && setting.hold <= configure_file.max_hold
	    && setting.hold >= configure_file.min_hold
	    && setting.brightness <= configure_file.max_brightness
	    && setting.brightness >= configure_file.min_brightness) { //check flash_disk
		beep(50); // disk ok
	} else { // disk error
		rewind(p_flash);
		fwrite(&default_setting, sizeof(flash_file), 1, p_flash);
		fflush(p_flash);

		beep(50, 50, 50, 50, 50, 50, 50, 50, 50);
	}

	p_uart = fopen("uart", "r");
	setvbuf(p_uart, nullptr, _IONBF, 0);

	p_lcd = fopen("lcd", "rb+");
	setvbuf(p_lcd, nullptr, _IOFBF, 16);

	p_dac = fopen("dac", "rb+");
	setvbuf(p_dac, nullptr, _IOFBF, sizeof(dac_file));

	p_pwm = fopen("pwm", "rb+");
	setvbuf(p_pwm, nullptr, _IOFBF, sizeof(pwm_file));

	p_ad8253 = fopen("ad8253", "rb+");
	setvbuf(p_ad8253, nullptr, _IOFBF, sizeof(ad8253_file));

	p_adc = fopen("adc", "rb");
	setvbuf(p_adc, nullptr, _IOFBF, sizeof(adc_file));

	encoder_open();
}
