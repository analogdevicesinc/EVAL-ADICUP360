/***************************************************************************//**
 *   @file   cmd_poll.cpp
 *   @brief  Implementation of cmd_poll.cpp
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
#include <hal/drivers/flash.h>
#include <hal/devices.h>
#include <hal/drivers/adc.h>

int cmd_poll(int argc, char *argv[])
{
	if (argc != 1) {
		printf("%s: syntax error, please use the following syntax: %s\n\n", argv[0], argv[0]);
	} else {
		flash_file flash_var;
		flash_file * p_flash_file = &flash_var;

		decltype(p_flash_file->voltage) voltage;
		fseek(p_flash, (int) (&p_flash_file->voltage) - (int) (p_flash_file), SEEK_SET);
		fread(&voltage, sizeof(flash_file::voltage), 1, p_flash);

		printf("EXC V: %fV\n", voltage);

		decltype(p_flash_file->frequency) frequency;
		fseek(p_flash, (int) (&p_flash_file->frequency) - (int) (p_flash_file),
		      SEEK_SET);
		fread(&frequency, sizeof(flash_file::frequency), 1, p_flash);

		printf("EXC FREQ: %fHz\n", frequency);

		decltype(p_flash_file->setup) setup;
		fseek(p_flash, (int) (&p_flash_file->setup) - (int) (p_flash_file), SEEK_SET);
		fread(&setup, sizeof(flash_file::setup), 1, p_flash);

		printf("EXC setup time: %f%%\n", setup);

		decltype(p_flash_file->hold) hold;
		fseek(p_flash, (int) (&p_flash_file->hold) - (int) (p_flash_file), SEEK_SET);
		fread(&hold, sizeof(flash_file::hold), 1, p_flash);

		printf("EXC hold time: %f%%\n", hold);

		decltype(p_flash_file->temp_coef) temp_coef;
		fseek(p_flash, (int) (&p_flash_file->temp_coef) - (int) (p_flash_file),
		      SEEK_SET);
		fread(&temp_coef, sizeof(flash_file::temp_coef), 1, p_flash);

		printf("TEMP COEF: %f%%/'C\n", temp_coef);

		decltype(p_flash_file->cell_const) cell_const;
		fseek(p_flash, (int) (&p_flash_file->cell_const) - (int) (p_flash_file),
		      SEEK_SET);
		fread(&cell_const, sizeof(flash_file::cell_const), 1, p_flash);

		printf("cell K: %f/cm\n", cell_const);

		adc_file adc;

		rewind(p_adc);
		fread(&adc, sizeof(adc_file), 1, p_adc);

		printf("ADC0 hits: %lu\n", adc.adc0_hit);

		printf("+I gain: %i\n", adc.p_curt_gain);
		printf("+Ip-p: %eA\n", adc.p_curt);

		printf("-I gain: %i\n", adc.n_curt_gain);
		printf("-Ip-p: %eA\n", adc.n_curt);

		printf("+V gain: %i\n", adc.p_volt_gain);
		printf("+Vp-p: %eV\n", adc.p_volt);

		printf("-V gain: %i\n", adc.n_volt_gain);
		printf("-Vp-p: %eV\n", adc.n_volt);

		printf("ADC1 hits: %lu\n", adc.adc1_hit);

		printf("RTD: PT%i\n", adc.rtd_type);
		printf("RTD wire: %i wire\n", adc.wire_mode);
		printf("TEMP: %f'C\n\n", adc.temp);

		float conductivity = (cell_const * (adc.p_curt - adc.n_curt) /
				      (adc.p_volt - adc.n_volt))
				     * (100 / (100 + temp_coef * (adc.temp - 25)));

		printf("conductivity: %eS/cm\n\n", conductivity);
	}

	fflush(stdout);
}
