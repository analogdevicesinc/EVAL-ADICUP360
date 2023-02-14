/***************************************************************************//**
 *   @file   cmd_setuptime.cpp
 *   @brief  Implementation of cmd_setuptime.cpp
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
#include <cstdio>
#include <hal/drivers/flash.h>
#include <hal/devices.h>
#include <hal/drivers/pwm.h>

int cmd_setuptime(int argc, char *argv[])
{
	if (argc != 2) {
		printf("%s: error, syntax: ADDRESS %s SETUP_TIME\n\n", argv[0], argv[0]);
	} else {
		pwm_file pwm;
		pwm.setup = -1;

		sscanf(argv[1], "%f", &pwm.setup);

		if (pwm.setup <= configure_file.max_setup
		    && pwm.setup >= configure_file.min_setup) {
			flash_file * p_flash_file;

			fseek(p_pwm, (int) (&pwm.setup) - (int) (&pwm), SEEK_SET);
			fwrite(&pwm.setup, sizeof(pwm_file::setup), 1, p_pwm);
			fflush(p_pwm);

			decltype(p_flash_file->setup) setup = pwm.setup;
			fseek(p_flash, (int) (&p_flash_file->setup) - (int) (p_flash_file), SEEK_SET);
			fwrite(&setup, sizeof(flash_file::setup), 1, p_flash);
			fflush(p_flash);

			printf("%s: the setup time is set to: %f%%\n\n", argv[0], pwm.setup);
		} else {
			printf("%s: error, setup time: %s is not in range: %f~%f\n\n", argv[0], argv[1],
			       configure_file.min_setup,
			       configure_file.max_setup);
		}
	}

	fflush(stdout);
}
