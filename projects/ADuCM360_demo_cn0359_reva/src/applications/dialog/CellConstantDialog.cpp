/***************************************************************************//**
 *   @file   CellConstantDialog.cpp
 *   @brief  Implementation of CellConstantDialog.cpp
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
#include <applications/dialog/CellConstantDialog.h>
#include <hal/devices.h>
#include <hal/drivers/buzzer.h>
#include <applications/dialog/SettingDialog.h>

CCellConstantDialog CellConstantDialog;

void CCellConstantDialog::OnCreat(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);

	rewind(p_lcd);
	fputs("Cell constant:", p_lcd);

	fseek(p_lcd, 48, SEEK_SET);
	fputs("  Save   Cancel", p_lcd);

	in_edit = false;
	menu_item = c10;

	flash_file * p_flash_file;

	fseek(p_flash, (int) (&p_flash_file->cell_const) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&constant, sizeof(flash_file::cell_const), 1, p_flash);

	OnDraw();
}

void CCellConstantDialog::OnEncoderCW(void)
{
	if (in_edit) {
		switch (menu_item) {
		default:
			menu_item = c10;
		case c10:
			constant += 10;
			break;
		case c1:
			constant += 1;
			break;
		case c0_1:
			constant += 0.1f;
			break;
		case c0_01:
			constant += 0.01f;
			break;
		case c0_001:
			constant += 0.001f;
			break;
		case c0_0001:
			constant += 0.0001f;
			break;
		}

		if (constant > configure_file.max_cell_const) {
			constant = configure_file.max_cell_const;
			beep(10);
		}
	} else {
		switch (menu_item) {
		default:
		case c10:
			menu_item = c1;
			break;
		case c1:
			menu_item = c0_1;
			break;
		case c0_1:
			menu_item = c0_01;
			break;
		case c0_01:
			menu_item = c0_001;
			break;
		case c0_001:
			menu_item = c0_0001;
			break;
		case c0_0001:
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

void CCellConstantDialog::OnEncoderCCW(void)
{
	if (in_edit) {
		switch (menu_item) {
		default:
			menu_item = c10;
		case c10:
			constant -= 10;
			break;
		case c1:
			constant -= 1;
			break;
		case c0_1:
			constant -= 0.1f;
			break;
		case c0_01:
			constant -= 0.01f;
			break;
		case c0_001:
			constant -= 0.001f;
			break;
		case c0_0001:
			constant -= 0.0001f;
			break;
		}

		if (constant < configure_file.min_cell_const) {
			constant = configure_file.min_cell_const;
			beep(10);
		}
	} else {
		switch (menu_item) {
		case c10:
			beep(10);
			return;
			break;
		case c1:
			menu_item = c10;
			break;
		case c0_1:
			menu_item = c1;
			break;
		case c0_01:
			menu_item = c0_1;
			break;
		case c0_001:
			menu_item = c0_01;
			break;
		case c0_0001:
			menu_item = c0_001;
			break;
		case save:
			menu_item = c0_0001;
			break;
		default:
		case cancel:
			menu_item = save;
			break;
		}
	}

	OnDraw();
}

void CCellConstantDialog::OnButtonDown(void)
{
	flash_file * p_flash_file;

	switch (menu_item) {
	default:
		menu_item = c10;
	case c10:
	case c1:
	case c0_1:
	case c0_01:
	case c0_001:
	case c0_0001:
		in_edit = !in_edit;
		break;
	case save:
		fseek(p_flash, (int) (&p_flash_file->cell_const) - (int) (p_flash_file),
		      SEEK_SET);
		fwrite(&constant, sizeof(flash_file::cell_const), 1, p_flash);
		fflush(p_flash);
	case cancel:
		p_dialog = &SettingDialog;
		p_dialog->OnCreat();
		return;
		break;
	}

	OnDraw();
}

void CCellConstantDialog::OnDraw(void)
{
	fseek(p_lcd, 99, SEEK_SET);
	fwrite(zeros, 7, 1, p_lcd);

	fseek(p_lcd, 112, SEEK_SET);
	fwrite(zeros, 16, 1, p_lcd);

	fseek(p_lcd, 34, SEEK_SET);

	fprintf(p_lcd, "%8.4f/cm", constant);

	switch (menu_item) {
	default:
		menu_item = c10;
	case c10:
		fseek(p_lcd, 99, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
		break;
	case c1:
		fseek(p_lcd, 100, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
		break;
	case c0_1:
		fseek(p_lcd, 102, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
		break;
	case c0_01:
		fseek(p_lcd, 103, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
		break;
	case c0_001:
		fseek(p_lcd, 104, SEEK_SET);
		fputc(0x1 + in_edit, p_lcd);
		break;
	case c0_0001:
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
