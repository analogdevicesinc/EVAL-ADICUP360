/***************************************************************************//**
 *   @file   ad8253.cpp
 *   @brief  Implementation of ad8253.cpp
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

#include <ADuCM360.h>
#include <errno.h>
#include <hal/drivers/ad8253.h>
#include <cstring>

static int position;

static int i_gain, v_gain;

ssize_t ad8253_read(void *buf, size_t count)
{
	ad8253_file file;

	file.i_gain = i_gain;
	file.v_gain = v_gain;

	if ((position + count) > sizeof(ad8253_file)) {
		count = sizeof(ad8253_file) - position;
	}

	memcpy(buf, (char*) (&file) + position, count);

	position += count;

	return count;
}

ssize_t ad8253_write(const void *buf, size_t count)
{
	ad8253_file file;

	file.i_gain = i_gain;
	file.v_gain = v_gain;

	if ((position + count) > sizeof(ad8253_file)) {
		count = sizeof(ad8253_file) - position;
	}

	memcpy((char*) (&file) + position, buf, count);

	if (file.v_gain != v_gain) {
		switch (file.v_gain) {
		case 1:
			GP1CLR_CLR2_BBA = true;
			GP2CLR_CLR1_BBA = true;
			break;
		case 10:
			GP1SET_SET2_BBA = true;
			GP2CLR_CLR1_BBA = true;
			break;
		case 100:
			GP1CLR_CLR2_BBA = true;
			GP2SET_SET1_BBA = true;
			break;
		case 1000:
			GP1SET_SET2_BBA = true;
			GP2SET_SET1_BBA = true;
			break;
		default:
			goto end_v;
			break;
		}
		GP0CLR_CLR0_BBA = true;
		GP0SET_SET0_BBA = true;
		v_gain = file.v_gain;
end_v:
		;
	}

	if (file.i_gain != i_gain) {
		switch (file.i_gain) {
		case 1:
			GP1CLR_CLR2_BBA = true;
			GP2CLR_CLR1_BBA = true;
			break;
		case 10:
			GP1SET_SET2_BBA = true;
			GP2CLR_CLR1_BBA = true;
			break;
		case 100:
			GP1CLR_CLR2_BBA = true;
			GP2SET_SET1_BBA = true;
			break;
		case 1000:
			GP1SET_SET2_BBA = true;
			GP2SET_SET1_BBA = true;
			break;
		default:
			goto end_i;
			break;
		}
		GP2CLR_CLR0_BBA = true;
		GP2SET_SET0_BBA = true;
		i_gain = file.i_gain;
end_i:
		;
	}

	position += count;

	return count;
}

off_t ad8253_lseek(off_t offset, int whence)
{
	switch (whence) {
	case SEEK_SET:
		if (offset > sizeof(ad8253_file) || offset < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = offset;
		}
		break;
	case SEEK_CUR:
		if ((position + offset) > sizeof(ad8253_file) || (position + offset) < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position += offset;
		}
		break;
	case SEEK_END:
		if (offset > 0 || (sizeof(ad8253_file) + offset) < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = sizeof(ad8253_file) + offset;
		}
		break;
	default:
		errno = ENXIO;
		return -1;
		break;
	}

	return position;
}

void ad8253_open(void)
{
	pADI_GP0->GPCON = (pADI_GP0->GPCON & ~GP0CON_CON0_MSK) | GP0CON_CON0_GPIO;
	pADI_GP1->GPCON = (pADI_GP1->GPCON & ~GP1CON_CON2_MSK) | GP1CON_CON2_GPIO;
	pADI_GP2->GPCON = (pADI_GP2->GPCON & ~GP2CON_CON0_MSK) | GP2CON_CON0_GPIO;
	pADI_GP2->GPCON = (pADI_GP2->GPCON & ~GP2CON_CON1_MSK) | GP2CON_CON1_GPIO;

	GP0OCE_OCE0_BBA = false;
	GP1OCE_OCE2_BBA = false;
	GP2OCE_OCE0_BBA = false;
	GP2OCE_OCE1_BBA = false;

	GP1CLR_CLR2_BBA = true;
	GP2CLR_CLR1_BBA = true;

	GP0SET_SET0_BBA = true;
	GP2SET_SET0_BBA = true;

	GP0OEN_OEN0_BBA = true;
	GP1OEN_OEN2_BBA = true;
	GP2OEN_OEN0_BBA = true;
	GP2OEN_OEN1_BBA = true;

	GP0PUL_PUL0_BBA = true;
	GP1PUL_PUL2_BBA = true;
	GP2PUL_PUL0_BBA = true;
	GP2PUL_PUL1_BBA = true;

	GP0CLR_CLR0_BBA = true; //write 0
	GP0SET_SET0_BBA = true;
	v_gain = 1;

	GP2CLR_CLR0_BBA = true; //write 0
	GP2SET_SET0_BBA = true;
	i_gain = 1;
}
