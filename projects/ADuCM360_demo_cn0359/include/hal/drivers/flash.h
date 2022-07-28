/***************************************************************************//**
 *   @file   flash.h
 *   @brief  Implementation of flash.h
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

#ifndef __FLASH_H__
#define __FLASH_H__

#include <unistd.h>

extern ssize_t flash_write(const void *buf, size_t count);

extern ssize_t flash_read(void *buf, size_t count);

extern off_t flash_lseek(off_t offset, int whence);

extern void flash_open(void);

#ifndef __TIMESTAMP__
#define __TIMESTAMP__
#endif

#ifndef __VERSION__
#define __VERSION__
#endif

#define CN0359_VERSION "<" __TIME__ ">, <" __DATE__ ">, <" __VERSION__ ">, <" __TIMESTAMP__ ">, " "<Circuits from the Lab>, " "<Copyright 2016, Analog Devices, Inc. All rights reserved.> "

struct flash_file {
	int baud_rate = 115200;
	int rs485_address = 30;
	float voltage = 1.f;
	float frequency = 100.f;
	float temp_coef = 2.f;
	float cell_const = 1.f;
	float setup = 10.f;
	float hold = 1.f;
	int brightness = 32;
} const default_setting;

struct {
	const int min_baud_rate = 9600;
	const int max_baud_rate = 460800;

	const int min_rs485_address = 1;
	const int max_rs485_address = 254;

	const float min_voltage = 0.1f;
	const float max_voltage = 10.f;

	const float min_freq = 10.f;
	const float max_freq = 100.e3f;

	const float min_temp_coef = -10.f;
	const float max_temp_coef = 10.f;

	const float min_cell_const = 0.01f;
	const float max_cell_const = 100.f;

	const float min_setup = 0.f;
	const float max_setup = 80.f;

	const float min_hold = 0.f;
	const float max_hold = 10.f;

	const int min_brightness = 0;
	const int max_brightness = 63;

	const char version[sizeof(CN0359_VERSION)] = CN0359_VERSION;
} const configure_file;

const char zeros[512] = {};

#endif
