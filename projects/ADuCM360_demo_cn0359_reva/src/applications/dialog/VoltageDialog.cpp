/***************************************************************************//**
 *   @file   VoltageDialog.cpp
 *   @brief  Implementation of VoltageDialog.cpp
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
#include <applications/dialog/VoltageDialog.h>
#include <hal/devices.h>
#include <cstdio>
#include <applications/dialog/SettingDialog.h>
#include <hal/drivers/buzzer.h>
#include <hal/drivers/dac.h>

CVoltageDialog VoltageDialog;

void CVoltageDialog::OnCreat(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);

	rewind(p_lcd);
	fputs("EXC voltage:", p_lcd);

	fseek(p_lcd, 48, SEEK_SET);
	fputs("  Save   Cancel", p_lcd);

	in_edit = false;
	menu_item = v1;

	flash_file * p_flash_file;
	fseek(p_flash, (int) (&p_flash_file->voltage) - (int) (p_flash_file), SEEK_SET);
	fread(&voltage, sizeof(flash_file::voltage), 1, p_flash);

	OnDraw();
}

void CVoltageDialog::OnEncoderCW(void)
{
	if (in_edit) {
		if (menu_item == v1) {
			voltage += 1;
		} else {
			voltage += 0.1f;
		}

		if (voltage > configure_file.max_voltage) {
			voltage = configure_file.max_voltage;
			beep(10);
		}

		dac_file dac;
		dac.voltage = voltage;
		fseek(p_dac, (int) (&dac.voltage) - (int) (&dac), SEEK_SET);
		fwrite(&dac.voltage, sizeof(dac_file::voltage), 1, p_dac);
		fflush(p_dac);
	} else {
		switch (menu_item) {
		default:
		case v1:
			menu_item = v0_1;
			break;
		case v0_1:
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

void CVoltageDialog::OnEncoderCCW(void)
{
	if (in_edit) {
		if (menu_item == v1) {
			voltage -= 1;
		} else {
			voltage -= 0.1f;
		}

		if (voltage < configure_file.min_voltage) {
			voltage = configure_file.min_voltage;
			beep(10);
		}

		dac_file dac;
		dac.voltage = voltage;
		fseek(p_dac, (int) (&dac.voltage) - (int) (&dac), SEEK_SET);
		fwrite(&dac.voltage, sizeof(dac_file::voltage), 1, p_dac);
		fflush(p_dac);
	} else {
		switch (menu_item) {
		case v1:
			beep(10);
			return;
			break;
		case v0_1:
			menu_item = v1;
			break;
		case save:
			menu_item = v0_1;
			break;
		default:
		case cancel:
			menu_item = save;
			break;
		}
	}

	OnDraw();
}

void CVoltageDialog::OnButtonDown(void)
{
	flash_file * p_flash_file;

	switch (menu_item) {
	default:
		menu_item = v1;
	case v1:
	case v0_1:
		in_edit = !in_edit;
		break;
	case save:
		fseek(p_flash, (int) (&p_flash_file->voltage) - (int) (p_flash_file), SEEK_SET);
		fwrite(&voltage, sizeof(flash_file::voltage), 1, p_flash);
		fflush(p_flash);

		p_dialog = &SettingDialog;
		p_dialog->OnCreat();
		return;
		break;
	case cancel:
		fseek(p_flash, (int) (&p_flash_file->voltage) - (int) (p_flash_file), SEEK_SET);
		fread(&voltage, sizeof(flash_file::voltage), 1, p_flash);

		dac_file dac;
		dac.voltage = voltage;
		fseek(p_dac, (int) (&dac.voltage) - (int) (&dac), SEEK_SET);
		fwrite(&dac.voltage, sizeof(dac_file::voltage), 1, p_dac);
		fflush(p_dac);

		p_dialog = &SettingDialog;
		p_dialog->OnCreat();
		return;
		break;
	}

	OnDraw();
}

void CVoltageDialog::OnDraw(void)
{
	fseek(p_lcd, 102, SEEK_SET);
	fwrite(zeros, 3, 1, p_lcd);

	fseek(p_lcd, 112, SEEK_SET);
	fwrite(zeros, 16, 1, p_lcd);

	fseek(p_lcd, 37, SEEK_SET);

	fprintf(p_lcd, "%4.1fV", voltage);

	switch (menu_item) {
	default:
		menu_item = v1;
	case v1:
		fseek(p_lcd, 102, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
		break;
	case v0_1:
		fseek(p_lcd, 104, SEEK_SET);
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
