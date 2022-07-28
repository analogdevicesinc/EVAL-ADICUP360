/***************************************************************************//**
 *   @file   pwm.cpp
 *   @brief  Implementation of pwm.cpp
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
#include <hal/devices.h>
#include <cmath>
#include <cassert>
#include <errno.h>
#include <cstring>
#include <hal/drivers/pwm.h>
#include <hal/drivers/flash.h>

static int position = 0;

static pwm_file pwm;

off_t pwm_lseek(off_t offset, int whence)
{
	switch (whence) {
	case SEEK_SET:
		if (offset > sizeof(pwm_file) || offset < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = offset;
		}
		break;
	case SEEK_CUR:
		if ((position + offset) > sizeof(pwm_file) || (position + offset) < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position += offset;
		}
		break;
	case SEEK_END:
		if (offset > 0 || sizeof(pwm_file) + offset < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = sizeof(pwm_file) + offset;
		}
		break;
	default:
		errno = ENXIO;
		return -1;
		break;
	}

	return position;
}

/*                             UCLK              UCLK
 * PWM frequency F = ──────────────────────── = ──────
 *                    2^^PRE * 2 * (LEN + 1)      DIV
 * DIV = 2^^(PRE + 1) * (LEN + 1)
 * DIV / 2^^(PRE + 1) <= 0x10000
 *
 */
static void pwm_update(const pwm_file &file)
{
	pwm = file;

	float div = SystemCoreClock / file.freq;

	assert(div >= 8);

	int pre = 0;

	while (div / (0x2 << pre) > 0x10000) {
		++pre;
	}

	assert(pre <= 0x7);

	int len = lround(div / (0x2 << pre));

	PWMCON0_ENABLE_BBA = false;

	pADI_PWM->PWMCON0 = (pADI_PWM->PWMCON0 & ~PWMCON0_PRE_MSK) | (pre << 6);

	pADI_PWM->PWM0LEN = pADI_PWM->PWM1LEN = pADI_PWM->PWM2LEN = len - 1;

	pADI_PWM->PWM0COM2 = pADI_PWM->PWM0LEN >> 1; //pwm1 50% duty

	pADI_PWM->PWM1COM1 = lround(pADI_PWM->PWM0COM2 * file.setup / 100.f); //setup
	pADI_PWM->PWM1COM0 = pADI_PWM->PWM0COM2 - lround(pADI_PWM->PWM0COM2 *
			     file.hold / 100.f); //hold

	pADI_PWM->PWM2COM1 = pADI_PWM->PWM0COM2 + pADI_PWM->PWM1COM1; //setup
	pADI_PWM->PWM2COM0 = pADI_PWM->PWM2LEN - (pADI_PWM->PWM0COM2 -
			     pADI_PWM->PWM1COM0); //hold

	PWMCON0_PWM1INV_BBA = file.inv;

	PWMCON0_SYNC_BBA = true;

	PWMCON0_ENABLE_BBA = true;

	GP1TGL_TGL2_BBA = true;

	GP1TGL_TGL2_BBA = true;

	PWMCON0_ENABLE_BBA = false;

	PWMCON0_SYNC_BBA = false;

	PWMCON0_ENABLE_BBA = true;

	PWMCON0_LCOMP_BBA = true;
}

ssize_t pwm_read(void *buf, size_t count)
{
	if ((position + count) > sizeof(pwm_file)) {
		count = sizeof(pwm_file) - position;
	}

	memcpy(buf, (char*) (&pwm) + position, count);

	position += count;

	return count;
}

ssize_t pwm_write(const void *buf, size_t count)
{
	pwm_file file = pwm;

	if ((position + count) > sizeof(pwm_file)) {
		count = sizeof(pwm_file) - position;
	}

	memcpy((char*) (&file) + position, buf, count);

	position += count;

	if (file.freq <= configure_file.max_freq && file.freq >= configure_file.min_freq
	    && file.setup <= configure_file.max_setup
	    && file.setup >= configure_file.min_setup
	    && file.hold <= configure_file.max_hold
	    && file.hold >= configure_file.min_hold) {
		pwm_update(file);
	}

	return count;
}

void pwm_open(void)
{
	CLKDIS_DISPWMCLK_BBA = false;
	pADI_CLKCTL->CLKCON1 = (pADI_CLKCTL->CLKCON1 & ~CLKCON1_PWMCD_MSK) |
			       CLKCON1_PWMCD_DIV1;

	pADI_GP1->GPCON = (pADI_GP1->GPCON & ~GP1CON_CON0_MSK) | GP1CON_CON0_PWMSYNC;
	pADI_GP1->GPCON = (pADI_GP1->GPCON & ~GP1CON_CON3_MSK) | GP1CON_CON3_PWM1;
	pADI_GP1->GPCON = (pADI_GP1->GPCON & ~GP1CON_CON4_MSK) | GP1CON_CON4_PWM2;
	pADI_GP1->GPCON = (pADI_GP1->GPCON & ~GP1CON_CON6_MSK) | GP1CON_CON6_PWM4;
	pADI_GP1->GPCON = (pADI_GP1->GPCON & ~GP1CON_CON2_MSK) | GP1CON_CON2_GPIO;

	GP1OCE_OCE0_BBA = false;
	GP1OCE_OCE3_BBA = false;
	GP1OCE_OCE4_BBA = false;
	GP1OCE_OCE6_BBA = false;
	GP1OCE_OCE2_BBA = false;

	GP1OEN_OEN0_BBA = true;
	GP1OEN_OEN3_BBA = true;
	GP1OEN_OEN4_BBA = true;
	GP1OEN_OEN6_BBA = true;
	GP1OEN_OEN2_BBA = true;

	GP1PUL_PUL0_BBA = true;
	GP1PUL_PUL3_BBA = true;
	GP1PUL_PUL4_BBA = true;
	GP1PUL_PUL6_BBA = true;
	GP1PUL_PUL2_BBA = true;

	PWMCON0_ENABLE_BBA = false;

	pADI_PWM->PWMCON0 = PWMCON0_SYNC_DIS | PWMCON0_PWM5INV_DIS | PWMCON0_PWM3INV_DIS
			    | PWMCON0_PWM1INV_DIS
			    | PWMCON0_PWMIEN_DIS | PWMCON0_ENA_DIS | PWMCON0_POINV_DIS | PWMCON0_HOFF_DIS |
			    PWMCON0_LCOMP_DIS
			    | PWMCON0_DIR_DIS | PWMCON0_MOD_DIS | PWMCON0_ENABLE_DIS;

	PWMCON1_TRIPEN_BBA = false;

	pADI_PWM->PWM0COM0 = 0x0; //pwm0 always high
	pADI_PWM->PWM0COM1 = 0xffff; //pwm0 always high
	pADI_PWM->PWM1COM2 = 0x0; //pwm3 not use
	pADI_PWM->PWM2COM2 = 0x0; //pwm5 not use

	flash_file * p_flash_file;

	decltype(flash_file::frequency) frequency;

	fseek(p_flash, (int) (&p_flash_file->frequency) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&frequency, sizeof(flash_file::frequency), 1, p_flash);

	pwm.freq = frequency;

	decltype(flash_file::setup) setup;

	fseek(p_flash, (int) (&p_flash_file->setup) - (int) (p_flash_file), SEEK_SET);
	fread(&setup, sizeof(flash_file::setup), 1, p_flash);

	pwm.setup = setup;

	decltype(flash_file::hold) hold;

	fseek(p_flash, (int) (&p_flash_file->hold) - (int) (p_flash_file), SEEK_SET);
	fread(&hold, sizeof(flash_file::hold), 1, p_flash);

	pwm.hold = hold;

	pwm.inv = false;

	pwm_update(pwm);
}

