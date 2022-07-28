/***************************************************************************//**
 *   @file   HoldTimeDialog.cpp
 *   @brief  Implementation of HoldTimeDialog.cpp
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

#include <applications/dialog/SettingDialog.h>
#include <applications/dialog/HoldTimeDialog.h>
#include <cstdio>
#include <hal/devices.h>
#include <hal/drivers/buzzer.h>
#include <hal/drivers/pwm.h>

CHoldTimeDialog HoldTimeDialog;

void CHoldTimeDialog::OnCreat(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);

	rewind(p_lcd);
	fputs("Hold time:", p_lcd);

	fseek(p_lcd, 48, SEEK_SET);
	fputs("  Save   Cancel", p_lcd);

	in_edit = false;
	menu_item = h10;

	flash_file * p_flash_file;
	fseek(p_flash, (int) (&p_flash_file->hold) - (int) (p_flash_file), SEEK_SET);
	fread(&hold_time, sizeof(flash_file::hold), 1, p_flash);

	OnDraw();
}

void CHoldTimeDialog::OnEncoderCW(void)
{
	if (in_edit) {
		switch (menu_item) {
		default:
			menu_item = h10;
		case h10:
			hold_time += 10;
			break;
		case h1:
			hold_time += 1;
			break;
		case h0_1:
			hold_time += 0.1f;
			break;
		case h0_01:
			hold_time += 0.01f;
			break;
		}

		if (hold_time > configure_file.max_hold) {
			hold_time = configure_file.max_hold;
			beep(10);
		}

		pwm_file pwm;

		pwm.hold = hold_time;

		fseek(p_pwm, (int) (&pwm.hold) - (int) (&pwm), SEEK_SET);
		fwrite(&pwm.hold, sizeof(pwm_file::hold), 1, p_pwm);
		fflush(p_pwm);
	} else {
		switch (menu_item) {
		default:
		case h10:
			menu_item = h1;
			break;
		case h1:
			menu_item = h0_1;
			break;
		case h0_1:
			menu_item = h0_01;
			break;
		case h0_01:
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

void CHoldTimeDialog::OnEncoderCCW(void)
{
	if (in_edit) {
		switch (menu_item) {
		default:
			menu_item = h10;
		case h10:
			hold_time -= 10;
			break;
		case h1:
			hold_time -= 1;
			break;
		case h0_1:
			hold_time -= 0.1f;
			break;
		case h0_01:
			hold_time -= 0.01f;
			break;
		}

		if (hold_time < configure_file.min_hold) {
			hold_time = configure_file.min_hold;
			beep(10);
		}

		pwm_file pwm;

		pwm.hold = hold_time;

		fseek(p_pwm, (int) (&pwm.hold) - (int) (&pwm), SEEK_SET);
		fwrite(&pwm.hold, sizeof(pwm_file::hold), 1, p_pwm);
		fflush(p_pwm);
	} else {
		switch (menu_item) {
		case h10:
			beep(10);
			return;
			break;
		case h1:
			menu_item = h10;
			break;
		case h0_1:
			menu_item = h1;
			break;
		case h0_01:
			menu_item = h0_1;
			break;
		case save:
			menu_item = h0_01;
			break;
		default:
		case cancel:
			menu_item = save;
			break;
		}
	}

	OnDraw();
}

void CHoldTimeDialog::OnButtonDown(void)
{
	flash_file * p_flash_file;

	pwm_file pwm;

	switch (menu_item) {
	default:
		menu_item = h10;
	case h10:
	case h1:
	case h0_1:
	case h0_01:
		in_edit = !in_edit;
		break;
	case save:
		fseek(p_flash, (int) (&p_flash_file->hold) - (int) (p_flash_file), SEEK_SET);
		fwrite(&hold_time, sizeof(flash_file::hold), 1, p_flash);
		fflush(p_flash);

		p_dialog = &SettingDialog;
		p_dialog->OnCreat();
		return;
		break;
	case cancel:
		fseek(p_flash, (int) (&p_flash_file->hold) - (int) (p_flash_file), SEEK_SET);
		fread(&hold_time, sizeof(flash_file::hold), 1, p_flash);

		pwm.hold = hold_time;
		fseek(p_pwm, (int) (&pwm.hold) - (int) (&pwm), SEEK_SET);
		fwrite(&pwm.hold, sizeof(pwm_file::hold), 1, p_pwm);
		fflush(p_pwm);

		p_dialog = &SettingDialog;
		p_dialog->OnCreat();
		return;
		break;
	}

	OnDraw();
}

void CHoldTimeDialog::OnDraw(void)
{
	fseek(p_lcd, 101, SEEK_SET);
	fwrite(zeros, 5, 1, p_lcd);

	fseek(p_lcd, 112, SEEK_SET);
	fwrite(zeros, 16, 1, p_lcd);

	fseek(p_lcd, 37, SEEK_SET);

	fprintf(p_lcd, "%5.2f%%", hold_time);

	switch (menu_item) {
	default:
		menu_item = h10;
	case h10:
		fseek(p_lcd, 101, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
		break;
	case h1:
		fseek(p_lcd, 102, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
		break;
	case h0_1:
		fseek(p_lcd, 104, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
		break;
	case h0_01:
		fseek(p_lcd, 105, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
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
