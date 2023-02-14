/***************************************************************************//**
 *   @file   dac.cpp
 *   @brief  Implementation of dac.cpp
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
#include <cmath>
#include <errno.h>
#include <cstring>
#include <hal/devices.h>
#include <hal/drivers/flash.h>
#include <hal/drivers/dac.h>

//schematic: (0x0fff0000 * 15k) / (1.2v * (15k + 110k)) = 26836992
#define GAIN 26836992.f

static int position = 0;

ssize_t dac_read(void *buf, size_t count)
{
	dac_file file;

	file.voltage = pADI_DAC->DACDAT / GAIN;

	if ((position + count) > sizeof(dac_file)) {
		count = sizeof(dac_file) - position;
	}

	memcpy(buf, (char*) (&file) + position, count);

	position += count;

	return count;
}

ssize_t dac_write(const void *buf, size_t count)
{
	dac_file file;

	file.voltage = pADI_DAC->DACDAT / GAIN;

	if ((position + count) > sizeof(dac_file)) {
		count = sizeof(dac_file) - position;
	}

	memcpy((char*) (&file) + position, buf, count);

	position += count;

	if (file.voltage <= configure_file.max_voltage
	    && file.voltage >= configure_file.min_voltage) {
		pADI_DAC->DACDAT = lround(file.voltage * GAIN);
	}

	return count;
}

off_t dac_lseek(off_t offset, int whence)
{
	switch (whence) {
	case SEEK_SET:
		if (offset > sizeof(dac_file) || offset < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = offset;
		}
		break;
	case SEEK_CUR:
		if ((position + offset) > sizeof(dac_file) || (position + offset) < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position += offset;
		}
		break;
	case SEEK_END:
		if (offset > 0 || (sizeof(dac_file) + offset) < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = sizeof(dac_file) + offset;
		}
		break;
	default:
		errno = ENXIO;
		return -1;
		break;
	}

	return position;
}

void dac_open(void)
{
	CLKDIS_DISDACCLK_BBA = false;

	pADI_ANA->REFCTRL = REFCTRL_REFPD_DIS;

	pADI_DAC->DACCON = DACCON_DMAEN_Off | DACCON_PD_DIS | DACCON_NPN_DIS |
			   DACCON_BUFBYP_DIS | DACCON_CLK_HCLK
			   | DACCON_CLR_On | DACCON_MDE_16BitFast |
			   DACCON_RNG_IntVref; //please check with user guide, DACCON_MDE_16BitFast is slow mode

	DACCON_CLR_BBA = true;

	flash_file * p_flash_file;

	decltype(flash_file::voltage) voltage;

	fseek(p_flash, (int) (&p_flash_file->voltage) - (int) (p_flash_file), SEEK_SET);
	fread(&voltage, sizeof(dac_file::voltage), 1, p_flash);

	pADI_DAC->DACDAT = lround(voltage * GAIN);
}
