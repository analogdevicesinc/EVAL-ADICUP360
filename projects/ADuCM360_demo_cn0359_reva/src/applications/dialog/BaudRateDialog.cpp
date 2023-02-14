/***************************************************************************//**
 *   @file   BaudRateDialog.cpp
 *   @brief  Implementation of BaudRateDialog.cpp
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
#include <applications/dialog/BaudRateDialog.h>
#include <cstdio>
#include <hal/devices.h>
#include <hal/drivers/buzzer.h>
#include <applications/dialog/SettingDialog.h>
#include <hal/drivers/uart.h>

CBaudRateDialog BaudRateDialog;

#define SIZE(a) (sizeof(a)/sizeof(a[0]))
static const int BAUD_RATE[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800};

void CBaudRateDialog::OnCreat(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);

	rewind(p_lcd);
	fputs("Baud rate:", p_lcd);

	fseek(p_lcd, 48, SEEK_SET);
	fputs("  Save   Cancel", p_lcd);

	in_edit = false;
	menu_item = br;

	flash_file * p_flash_file;
	fseek(p_flash, (int) (&p_flash_file->baud_rate) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&baud_rate, sizeof(flash_file::baud_rate), 1, p_flash);

	OnDraw();
}

void CBaudRateDialog::OnEncoderCW(void)
{
	if (in_edit) {
		for (int i = 0; i < SIZE(BAUD_RATE); ++i) {
			if (i == SIZE(BAUD_RATE) - 1) {
				beep(10);
				return;
			}

			if (baud_rate == BAUD_RATE[i]) {
				baud_rate = BAUD_RATE[i + 1];
				break;
			}
		}
	} else {
		switch (menu_item) {
		default:
		case br:
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

void CBaudRateDialog::OnEncoderCCW(void)
{
	if (in_edit) {
		for (int i = SIZE(BAUD_RATE) - 1; i >= 0; --i) {
			if (i == 0) {
				beep(10);
				return;
			}

			if (baud_rate == BAUD_RATE[i]) {
				baud_rate = BAUD_RATE[i - 1];
				break;
			}
		}
	} else {
		switch (menu_item) {
		case br:
			beep(10);
			return;
			break;
		case save:
			menu_item = br;
			break;
		default:
		case cancel:
			menu_item = save;
			break;
		}
	}

	OnDraw();
}

void CBaudRateDialog::OnButtonDown(void)
{
	flash_file * p_flash_file;

	switch (menu_item) {
	default:
		menu_item = br;
	case br:
		in_edit = !in_edit;
		break;
	case save:
		fseek(p_flash, (int) (&p_flash_file->baud_rate) - (int) (p_flash_file),
		      SEEK_SET);
		fwrite(&baud_rate, sizeof(flash_file::baud_rate), 1, p_flash);
		fflush(p_flash);
		uart_baud(baud_rate);
	case cancel:
		p_dialog = &SettingDialog;
		p_dialog->OnCreat();
		return;
		break;
	}

	OnDraw();
}

void CBaudRateDialog::OnDraw(void)
{
	fseek(p_lcd, 100, SEEK_SET);
	fwrite(zeros, 6, 1, p_lcd);

	fseek(p_lcd, 112, SEEK_SET);
	fwrite(zeros, 16, 1, p_lcd);

	fseek(p_lcd, 36, SEEK_SET);
	fprintf(p_lcd, "%6ibps", baud_rate);

	switch (menu_item) {
	default:
		menu_item = br;
	case br:
		fseek(p_lcd, 100, SEEK_SET);
		if (in_edit) {
			fputs("\2\2\2\2\2\2", p_lcd);
		} else {
			fputs("\1\1\1\1\1\1", p_lcd);
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

