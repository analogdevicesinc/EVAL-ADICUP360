/***************************************************************************//**
 *   @file   adc.cpp
 *   @brief  Implementation of adc.cpp
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
#include <hal/drivers/adc.h>
#include <cstring>
#include <errno.h>
#include <cstdio>
#include <applications/message.h>
#include <cstdlib>
#include <hal/devices.h>
#include <hal/drivers/ad8253.h>
#include <hal/timer.h>
#include <hal/drivers/pwm.h>
#include <hal/RTD.h>

#define ADC0FLT_AF (15<<8)
#define ADC0FLT_SF 124

#define ADC1FLT_AF (15<<8)
#define ADC1FLT_SF 124

/*  ad8253 output p-p voltage     Vref      7.5kΩ       3.3V         7.5kΩ
 * ─────────────────────────── = ─────── * ─────── = ──────────── * ─────── = 7.68341124057769775390625e-8
 *             data               2^^28     1.2kΩ     0x10000000     1.2kΩ
 */
#define ADC0_GAIN 7.68341124057769775390625e-8

static volatile int adc0_result,
       adc1_result; //transfer value from IRQ to on_adc?()
static int adc0_cfg = 0;

static adc_file result;

static int position = 0;

ssize_t adc_read(void *buf, size_t count)
{
	if ((position + count) > sizeof(adc_file)) {
		count = sizeof(adc_file) - position;
	}

	memcpy(buf, (char*) (&result) + position, count);

	position += count;

	return count;
}

ssize_t adc_write(const void *buf, size_t count)
{
	if ((position + count) > sizeof(adc_file)) {
		count = sizeof(adc_file) - position;
	}

	position += count;

	return count;
}

off_t adc_lseek(off_t offset, int whence)
{
	switch (whence) {
	case SEEK_SET:
		if (offset > sizeof(adc_file) || offset < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = offset;
		}
		break;
	case SEEK_CUR:
		if ((position + offset) > sizeof(adc_file) || (position + offset) < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position += offset;
		}
		break;
	case SEEK_END:
		if (offset > 0 || (sizeof(adc_file) + offset) < 0) {
			errno = EINVAL;
			return -1;
		} else {
			position = sizeof(adc_file) + offset;
		}
		break;
	default:
		errno = ENXIO;
		return -1;
		break;
	}

	return position;
}

static int start_adc0(int argc, char *argv[])
{
	pADI_ADC0->MDE = (pADI_ADC0->MDE & ~ADC0MDE_ADCMD_MSK) | ADC0MDE_ADCMD_SINGLE;
	ADC0CON_ADCEN_BBA = true;

	return 0;
}

void adc_open(void)
{
	CLKDIS_DISADCCLK_BBA = false;

	pADI_ANA->REFCTRL = REFCTRL_REFPD_DIS;

	pADI_ADC0->MDE = ADC0MDE_PGA_G1 | ADC0MDE_ADCMOD2_MOD2OFF |
			 ADC0MDE_ADCMD_SINGLE;

	pADI_ADC0->FLT = ADC0FLT_CHOP_ON | ADC0FLT_RAVG2_ON | ADC0FLT_SINC4EN_DIS |
			 ADC0FLT_AF | ADC0FLT_NOTCH2_EN | ADC0FLT_SF;

	pwm_file pwm;
	pwm.inv = false;
	fseek(p_pwm, (int) (&pwm.inv) - (int) (&pwm), SEEK_SET);
	fwrite(&pwm.inv, sizeof(pwm_file::inv), 1, p_pwm);
	fflush(p_pwm);

	pADI_ADC0->CON = ADC0CON_ADCEN_DIS | ADC0CON_ADCCODE_INT | ADC0CON_BUFPOWN_EN |
			 ADC0CON_BUFPOWP_EN
			 | ADC0CON_BUFBYPP_EN | ADC0CON_BUFBYPN_EN | ADC0CON_ADCREF_AVDDREF |
			 ADC0CON_ADCDIAG_DIAG_OFF
			 | ADC0CON_ADCCP_AIN0 | ADC0CON_ADCCN_AIN1;
	adc0_cfg = 0;
	result.adc0_hit = 0;
	result.adc1_hit = 0;
	result.temp = 25;
	result.wire_mode = -1;
	result.rtd_type = -1;

	pADI_ADC0->ADCCFG = ADCCFG_SIMU_DIS | ADCCFG_BOOST30_DIS | ADCCFG_PINSEL_DIS |
			    ADCCFG_GNDSWON_EN
			    | ADCCFG_GNDSWRESEN_DIS | ADCCFG_EXTBUF_OFF;

	pADI_ADC0->MSKI = ADC0MSKI_ATHEX_DIS | ADC0MSKI_THEX_DIS | ADC0MSKI_OVR_DIS |
			  ADC0MSKI_RDY_EN;

	pADI_ADC1->MDE = ADC1MDE_PGA_G1 | ADC1MDE_ADCMOD2_MOD2OFF |
			 ADC1MDE_ADCMD_SINGLE;

	pADI_ADC1->FLT = ADC1FLT_CHOP_ON | ADC1FLT_RAVG2_ON | ADC1FLT_SINC4EN_DIS |
			 ADC1FLT_AF | ADC1FLT_NOTCH2_EN
			 | ADC1FLT_SF;

	pADI_ADC1->CON = ADC1CON_ADCEN_DIS | ADC1CON_ADCCODE_UINT | ADC1CON_BUFPOWN_EN |
			 ADC1CON_BUFPOWP_EN
			 | ADC1CON_BUFBYPP_EN | ADC1CON_BUFBYPN_EN | ADC1CON_ADCREF_INTREF |
			 ADC1CON_ADCDIAG_DIAG_ALL
			 | ADC1CON_ADCCP_AIN6 | ADC1CON_ADCCN_AGND;

	pADI_ANA->IEXCCON = IEXCCON_PD_off | IEXCCON_REFSEL_Int | IEXCCON_IPSEL1_AIN7 |
			    IEXCCON_IPSEL0_Off;

	pADI_ANA->IEXCDAT = IEXCDAT_IDAT_600uA | IEXCDAT_IDAT0_EN; //610uA

	pADI_ADC1->ADCCFG = ADCSCFG1_SIMU_DIS | ADCSCFG1_BOOST30_DIS |
			    ADCSCFG1_PINSEL_DIS | ADCSCFG1_GNDSWON_EN
			    | ADCSCFG1_GNDSWRESEN_DIS | ADCSCFG1_EXTBUF_OFF;

	pADI_ADC1->MSKI = ADC1MSKI_ATHEX_DIS | ADC1MSKI_THEX_DIS | ADC1MSKI_OVR_DIS |
			  ADC1MSKI_RDY_EN;

	NVIC_SetPriority(ADC0_IRQn, NVIC_EncodePriority(6, 1, 1));

	NVIC_SetPriority(ADC1_IRQn, NVIC_EncodePriority(6, 1, 1));

	NVIC_EnableIRQ(ADC0_IRQn);

	NVIC_EnableIRQ(ADC1_IRQn);

	timer t;
	t.time = 50;
	t.timer_app.argc = 0;
	t.timer_app.fun = start_adc0;
	t.timer_app.argv = new char*;

	new_timer(t);

	ADC1CON_ADCEN_BBA = true;
}

/*
 * ┌──────────────┬──────┬───────┬──────────┐
 * |   adc0_cfg   │ IN+  │  IN-  │ pwm1 inv │
 * ├──────────────┼──────┼───────┼──────────┤
 * |       0      │ ADC0 │  ADC1 │    0     │
 * ├──────────────┼──────┼───────┼──────────┤
 * |       1      │ ADC2 │  ADC3 │    0     │
 * ├──────────────┼──────┼───────┼──────────┤
 * |       2      │ ADC0 │  ADC1 │    1     │
 * ├──────────────┼──────┼───────┼──────────┤
 * |       3      │ ADC2 │  ADC3 │    1     │
 * └──────────────┴──────┴───────┴──────────┘
 *
 * chopping setup time 50ms
 *
 */
static int on_adc0(int argc, char *argv[])
{
	ad8253_file ad8253;
	timer t;

	rewind(p_ad8253);
	fread(&ad8253, sizeof(ad8253_file), 1, p_ad8253);

	if (abs(adc0_result) > (0x10000000 * 0.94)) { //upper threshold is 0.94*FS
		if ((adc0_cfg & 0x1) == 0x0) { //current channel
			if (ad8253.i_gain > 1) {
				ad8253.i_gain /= 10;

				fseek(p_ad8253, (int) (&ad8253.i_gain) - (int) (&ad8253), SEEK_SET);

				fwrite(&ad8253.i_gain, sizeof(ad8253_file::i_gain), 1, p_ad8253);

				fflush(p_ad8253);

				t.time = 50;
				t.timer_app.argc = 0;
				t.timer_app.fun = start_adc0;
				t.timer_app.argv = new char*;

				new_timer(t);

				return 0;
			}
		} else { //voltage channel
			if (ad8253.v_gain > 1) {
				ad8253.v_gain /= 10;

				fseek(p_ad8253, (int) (&ad8253.v_gain) - (int) (&ad8253), SEEK_SET);

				fwrite(&ad8253.v_gain, sizeof(ad8253_file::v_gain), 1, p_ad8253);

				fflush(p_ad8253);

				t.time = 50;
				t.timer_app.argc = 0;
				t.timer_app.fun = start_adc0;
				t.timer_app.argv = new char*;

				new_timer(t);

				return 0;
			}
		}
	}

	if (abs(adc0_result) < (0x10000000 * 0.088)) { //lower threshold is 0.088*FS
		if ((adc0_cfg & 0x1) == 0x0) { //current channel
			if (ad8253.i_gain < 1000) {
				ad8253.i_gain *= 10;

				fseek(p_ad8253, (int) (&ad8253.i_gain) - (int) (&ad8253), SEEK_SET);

				fwrite(&ad8253.i_gain, sizeof(ad8253_file::i_gain), 1, p_ad8253);

				fflush(p_ad8253);

				t.time = 50;
				t.timer_app.argc = 0;
				t.timer_app.fun = start_adc0;
				t.timer_app.argv = new char*;

				new_timer(t);

				return 0;
			}
		} else { //voltage channel
			if (ad8253.v_gain < 1000) {
				ad8253.v_gain *= 10;

				fseek(p_ad8253, (int) (&ad8253.v_gain) - (int) (&ad8253), SEEK_SET);

				fwrite(&ad8253.v_gain, sizeof(ad8253_file::v_gain), 1, p_ad8253);

				fflush(p_ad8253);

				t.time = 50;
				t.timer_app.argc = 0;
				t.timer_app.fun = start_adc0;
				t.timer_app.argv = new char*;

				new_timer(t);

				return 0;
			}
		}
	}

	pwm_file pwm;

	switch (adc0_cfg) {
	default:
		adc0_cfg = 0;
	case 0:
		result.p_curt = ((adc0_result * ADC0_GAIN) / ad8253.i_gain) / 1000; // R47 = 1k
		result.p_curt_gain = ad8253.i_gain;

		pADI_ADC0->CON = (pADI_ADC0->CON & ~ADC0CON_ADCCP_MSK & ~ADC0CON_ADCCN_MSK) |
				 ADC0CON_ADCCP_AIN2
				 | ADC0CON_ADCCN_AIN3;
		adc0_cfg = 1;
		start_adc0(0, nullptr);
		break;
	case 1:
		result.p_volt = (adc0_result * ADC0_GAIN) / ad8253.v_gain;
		result.p_volt_gain = ad8253.v_gain;

		pwm.inv = true;
		fseek(p_pwm, (int) (&pwm.inv) - (int) (&pwm), SEEK_SET);
		fwrite(&pwm.inv, sizeof(pwm_file::inv), 1, p_pwm);
		fflush(p_pwm);

		pADI_ADC0->CON = (pADI_ADC0->CON & ~ADC0CON_ADCCP_MSK & ~ADC0CON_ADCCN_MSK) |
				 ADC0CON_ADCCP_AIN0
				 | ADC0CON_ADCCN_AIN1;
		adc0_cfg = 2;

		t.time = 50;
		t.timer_app.argc = 0;
		t.timer_app.fun = start_adc0;
		t.timer_app.argv = new char*;

		new_timer(t);
		break;
	case 2:
		result.n_curt = ((adc0_result * ADC0_GAIN) / ad8253.i_gain) / 1000; // R47 = 1k
		result.n_curt_gain = ad8253.i_gain;

		pADI_ADC0->CON = (pADI_ADC0->CON & ~ADC0CON_ADCCP_MSK & ~ADC0CON_ADCCN_MSK) |
				 ADC0CON_ADCCP_AIN2
				 | ADC0CON_ADCCN_AIN3;
		adc0_cfg = 3;
		start_adc0(0, nullptr);
		break;
	case 3:
		result.n_volt = (adc0_result * ADC0_GAIN) / ad8253.v_gain;
		result.n_volt_gain = ad8253.v_gain;

		pwm.inv = false;
		fseek(p_pwm, (int) (&pwm.inv) - (int) (&pwm), SEEK_SET);
		fwrite(&pwm.inv, sizeof(pwm_file::inv), 1, p_pwm);
		fflush(p_pwm);

		pADI_ADC0->CON = (pADI_ADC0->CON & ~ADC0CON_ADCCP_MSK & ~ADC0CON_ADCCN_MSK) |
				 ADC0CON_ADCCP_AIN0
				 | ADC0CON_ADCCN_AIN1;
		adc0_cfg = 0;

		t.time = 50;
		t.timer_app.argc = 0;
		t.timer_app.fun = start_adc0;
		t.timer_app.argv = new char*;

		new_timer(t);
		break;
	}

	++result.adc0_hit;

	app msg;
	msg.argc = 0;
	msg.fun = on_conductivity;
	msg.argv = new char*;
	post_message(msg);

	return 0;
}

/*
 * ┌─────────────┬────────────┐
 * | temperature │ resistance │
 * ├─────────────┼────────────┤
 * |    120°C    │   1.4607   │
 * ├─────────────┼────────────┤
 * |    -20°C    │   0.9216   │
 * └─────────────┴────────────┘
 *
 * unbuffer mode ADC input resistance:
 *
 *           1V
 * Radc = ───────── = 2MΩ
 *         500nA/V
 *
 *              2MΩ//RTD                   1074547130368 * RTD
 * ADC1DATA = ──────────── * 0x10000000 = ─────────────────────
 *             2MΩ//1.5kΩ                     6M + 3 * RTD
 *
 *               6M * ADC1DATA
 * RTD = ────────────────────────────────
 *        1074547130368 - (3 * ADC1DATA)
 *
 * for PT1000
 *
 *  120°C      2MΩ//1.4607kΩ
 * ADC1DATA = ─────────────── * 0x10000000 = 0x0f94c35c
 *              2MΩ//1.5kΩ
 *
 *  -20°C      2MΩ//921.6kΩ
 * ADC1DATA = ─────────────── * 0x10000000 = 0x09d54f53
 *              2MΩ//1.5kΩ
 *
 * for PT100
 *
 *  120°C      2MΩ//146.07Ω
 * ADC1DATA = ─────────────── * 0x10000000 = 0x018f2373
 *              2MΩ//1.5kΩ
 *
 *  -20°C       2MΩ//92.16Ω
 * ADC1DATA = ─────────────── * 0x10000000 = 0x00fbd5db
 *              2MΩ//1.5kΩ
 *
 */
static int on_adc1(int argc, char *argv[])
{
	static int wire = 4;

	if ((pADI_ADC1->CON & ADC1CON_ADCDIAG_MSK) !=
	    ADC1CON_ADCDIAG_DIAG_OFF) { //diagnostic mode
		if ((pADI_ADC1->CON & ADC1CON_ADCCP_MSK) ==
		    ADC1CON_ADCCP_AIN6) { //AIN6 diagnostic
			if (adc1_result < 0xfff0000) { //available
				wire = 4;
				pADI_ADC1->CON = (pADI_ADC1->CON & ~ADC1CON_ADCREF_MSK & ~ADC1CON_ADCDIAG_MSK &
						  ~ADC1CON_ADCCP_MSK
						  & ~ADC1CON_ADCCN_MSK) | ADC1CON_ADCREF_EXTREF2 | ADC1CON_ADCDIAG_DIAG_OFF |
						 ADC1CON_ADCCP_AIN6 | ADC1CON_ADCCN_AIN5;
				pADI_ANA->IEXCCON = (pADI_ANA->IEXCCON & ~IEXCCON_IPSEL0_MSK) |
						    IEXCCON_IPSEL0_Off;
			} else {
				pADI_ADC1->CON = (pADI_ADC1->CON & ~ADC1CON_ADCCP_MSK) | ADC1CON_ADCCP_AIN5;
			}
		} else { //AIN5 diagnostic
			if (adc1_result < 0xfff0000) { //available
				wire = 3;
				pADI_ADC1->CON = (pADI_ADC1->CON & ~ADC1CON_ADCREF_MSK & ~ADC1CON_ADCDIAG_MSK &
						  ~ADC1CON_ADCCP_MSK
						  & ~ADC1CON_ADCCN_MSK) | ADC1CON_ADCREF_EXTREF2 | ADC1CON_ADCDIAG_DIAG_OFF |
						 ADC1CON_ADCCP_AIN8 | ADC1CON_ADCCN_AIN5;
				pADI_ANA->IEXCCON = (pADI_ANA->IEXCCON & ~IEXCCON_IPSEL0_MSK) |
						    IEXCCON_IPSEL0_AIN5;
			} else {
				wire = 2;
				pADI_ADC1->CON = (pADI_ADC1->CON & ~ADC1CON_ADCREF_MSK & ~ADC1CON_ADCDIAG_MSK &
						  ~ADC1CON_ADCCP_MSK
						  & ~ADC1CON_ADCCN_MSK) | ADC1CON_ADCREF_EXTREF2 | ADC1CON_ADCDIAG_DIAG_OFF |
						 ADC1CON_ADCCP_AIN8 | ADC1CON_ADCCN_AIN9;
				pADI_ANA->IEXCCON = (pADI_ANA->IEXCCON & ~IEXCCON_IPSEL0_MSK) |
						    IEXCCON_IPSEL0_Off;
			}
		}
	} else { //normal mode
		if ((adc1_result >= 0x09d54f53 && adc1_result <= 0x0f94c35c)
		    || (adc1_result >= 0x00fbd5db && adc1_result <= 0x018f2373)) {
			if (adc1_result >= 0x09d54f53) { //PT1000
				result.temp = res2temp(6.e3f * adc1_result / (1074547130368ll -
						       (3ll * adc1_result))); //normalized to 1 at 0°C
				result.rtd_type = 1000;
			} else { //PT100
				result.temp = res2temp(6.e4f * adc1_result / (1074547130368ll -
						       (3ll * adc1_result))); //normalized to 1 at 0°C
				result.rtd_type = 100;
			}

			result.wire_mode = wire;
			++result.adc1_hit;
		} else { //data not reliable
			result.wire_mode = -1;
			result.rtd_type = -1;
			result.temp = 25;
		}

		app msg;
		msg.argc = 0;
		msg.fun = on_rtd;
		msg.argv = new char*;
		post_message(msg);

		pADI_ADC1->CON = (pADI_ADC1->CON & ~ADC1CON_ADCREF_MSK & ~ADC1CON_ADCDIAG_MSK &
				  ~ADC1CON_ADCCP_MSK & ~ADC1CON_ADCCN_MSK)
				 | ADC1CON_ADCREF_INTREF | ADC1CON_ADCDIAG_DIAG_ALL | ADC1CON_ADCCP_AIN6 |
				 ADC1CON_ADCCN_AGND; //enter diagnostic mode
	}

	pADI_ADC1->MDE = (pADI_ADC1->MDE & ~ADC1MDE_ADCMD_MSK) | ADC1MDE_ADCMD_SINGLE;
	ADC1CON_ADCEN_BBA = true;

	return 0;
}

#ifdef __cplusplus
extern "C"
{
#endif
void ADC0_Int_Handler(void)
{
	adc0_result = pADI_ADC0->DAT;

	app msg;
	msg.argc = 0;
	msg.fun = on_adc0;
	ts_post_message(msg);
}

void ADC1_Int_Handler(void)
{
	adc1_result = pADI_ADC1->DAT;

	app msg;
	msg.argc = 0;
	msg.fun = on_adc1;
	ts_post_message(msg);
}

#ifdef __cplusplus
}
#endif

