/***************************************************************************//**
 *   @file   FrequencyDialog.cpp
 *   @brief  Implementation of FrequencyDialog.cpp
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

#include <applications/dialog/FrequencyDialog.h>
#include <hal/devices.h>
#include <cstdio>
#include <cmath>
#include <hal/drivers/buzzer.h>
#include <applications/dialog/SettingDialog.h>
#include <hal/drivers/pwm.h>
#include <hal/drivers/flash.h>

CFrequencyDialog FrequencyDialog;

#define SIZE(a) (sizeof(a)/sizeof(a[0]))

static const int E24[] =
{10, 11, 12, 13, 15, 16, 18, 20, 22, 24, 27, 30, 33, 36, 39, 43, 47, 51, 56, 62, 68, 75, 82, 91};

CFrequencyDialog::CFrequencyDialog(void)
{
	max_multiple = pow(10, floor(log10(configure_file.max_freq)) - 1);
	max_base = lround(configure_file.max_freq / max_multiple);
	min_multiple = pow(10, floor(log10(configure_file.min_freq)) - 1);
	min_base = lround(configure_file.min_freq / min_multiple);
}

/*
 * frequency = base * multiple
 *
 */
void CFrequencyDialog::OnCreat(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);

	rewind(p_lcd);
	fputs("EXC frequency:", p_lcd);

	fseek(p_lcd, 48, SEEK_SET);
	fputs("  Save   Cancel", p_lcd);

	in_edit = false;
	menu_item = freq;

	pwm_file pwm;
	fseek(p_pwm, (int) (&pwm.freq) - (int) (&pwm), SEEK_SET);
	fread(&pwm.freq, sizeof(pwm_file::freq), 1, p_pwm);

	multiple = pow(10, floor(log10(pwm.freq)) - 1);
	base = lround(pwm.freq / multiple);

	OnDraw();
}

void CFrequencyDialog::OnEncoderCW(void)
{
	if (in_edit) {
		for (int i = 0; i < SIZE(E24); ++i) {
			if (i == SIZE(E24) - 1) {
				base = E24[0];
				multiple *= 10;
				break;
			}

			if (base < E24[i]) {
				base = E24[i];
				break;
			}
		}

		if ((base * multiple) > configure_file.max_freq) {
			base = max_base;
			multiple = max_multiple;
			beep(10);
		}

		pwm_file pwm;

		pwm.freq = base * multiple;

		fseek(p_pwm, (int) (&pwm.freq) - (int) (&pwm), SEEK_SET);
		fwrite(&pwm.freq, sizeof(pwm_file::freq), 1, p_pwm);
		fflush(p_pwm);
	} else {
		switch (menu_item) {
		default:
		case freq:
			menu_item = save;
			break;
		case save:
			menu_item = cancel;
			break;
		case cancel:
			beep(10);
			return;
			break;
		}
	}

	OnDraw();
}

void CFrequencyDialog::OnEncoderCCW(void)
{
	if (in_edit) {
		for (int i = SIZE(E24) - 1; i >= 0; --i) {
			if (i == 0) {
				base = E24[SIZE(E24) - 1];
				multiple /= 10;
				break;
			}

			if (base > E24[i]) {
				base = E24[i];
				break;
			}
		}

		if ((base * multiple) < configure_file.min_freq) {
			base = min_base;
			multiple = min_multiple;
			beep(10);
		}

		pwm_file pwm;

		pwm.freq = base * multiple;

		fseek(p_pwm, (int) (&pwm.freq) - (int) (&pwm), SEEK_SET);
		fwrite(&pwm.freq, sizeof(pwm_file::freq), 1, p_pwm);
		fflush(p_pwm);
	} else {
		switch (menu_item) {
		case freq:
			beep(10);
			return;
			break;
		case save:
			menu_item = freq;
			break;
		default:
		case cancel:
			menu_item = save;
			break;
		}
	}

	OnDraw();
}

void CFrequencyDialog::OnButtonDown(void)
{
	flash_file * p_flash_file;

	decltype(flash_file::frequency) frequency;

	pwm_file pwm;

	switch (menu_item) {
	default:
		menu_item = freq;
	case freq:
		in_edit = !in_edit;
		break;
	case save:
		pwm.freq = base * multiple;

		frequency = pwm.freq;

		fseek(p_flash, (int) (&p_flash_file->frequency) - (int) (p_flash_file),
		      SEEK_SET);
		fwrite(&frequency, sizeof(flash_file::frequency), 1, p_flash);
		fflush(p_flash);

		p_dialog = &SettingDialog;
		p_dialog->OnCreat();
		return;
		break;
	case cancel:
		fseek(p_flash, (int) (&p_flash_file->frequency) - (int) (p_flash_file),
		      SEEK_SET);
		fread(&frequency, sizeof(flash_file::frequency), 1, p_flash);

		pwm.freq = frequency;

		fseek(p_pwm, (int) (&pwm.freq) - (int) (&pwm), SEEK_SET);
		fwrite(&pwm.freq, sizeof(pwm_file::freq), 1, p_pwm);
		fflush(p_pwm);

		p_dialog = &SettingDialog;
		p_dialog->OnCreat();
		return;
		break;
	}

	OnDraw();
}

void CFrequencyDialog::OnDraw(void)
{
	fseek(p_lcd, 100, SEEK_SET);
	fwrite(zeros, 8, 1, p_lcd);

	fseek(p_lcd, 112, SEEK_SET);
	fwrite(zeros, 16, 1, p_lcd);

	fseek(p_lcd, 36, SEEK_SET);
	fwrite(zeros, 8, 1, p_lcd);

	fseek(p_lcd, 36, SEEK_SET);

	if ((base * multiple) < 1000) {
		fprintf(p_lcd, "%5.1fHz", (float) (base * multiple));
	} else {
		fprintf(p_lcd, "%5.1fkHz", (base * multiple) / 1000.f);
	}

	switch (menu_item) {
	default:
		menu_item = freq;
	case freq:
		fseek(p_lcd, 100, SEEK_SET);
		if (in_edit) {
			fputs("\2\2\2\2\2\2\2\2", p_lcd);
		} else {
			fputs("\1\1\1\1\1\1\1\1", p_lcd);
		}
		break;
	case save:
		fseek(p_lcd, 112, SEEK_SET);
		fputs("\1\1\1\1\1\1\1\1", p_lcd);
		break;
	case cancel:
		fseek(p_lcd, 120, SEEK_SET);
		fputs("\1\1\1\1\1\1\1\1", p_lcd);
		break;
	}

	fflush(p_lcd);
}
