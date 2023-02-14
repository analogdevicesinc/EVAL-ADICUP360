/***************************************************************************//**
 *   @file   AddressDialog.cpp
 *   @brief  Implementation of AddressDialog.cpp
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
#include <applications/dialog/AddressDialog.h>
#include <cstdio>
#include <hal/devices.h>
#include <hal/drivers/buzzer.h>
#include <applications/dialog/SettingDialog.h>

CAddressDialog AddressDialog;

void CAddressDialog::OnCreat(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);

	rewind(p_lcd);
	fputs("RS-485 Address:", p_lcd);

	fseek(p_lcd, 48, SEEK_SET);
	fputs("  Save   Cancel", p_lcd);

	in_edit = false;
	menu_item = adr;

	flash_file * p_flash_file;
	fseek(p_flash, (int) (&p_flash_file->rs485_address) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&address, sizeof(flash_file::rs485_address), 1, p_flash);

	OnDraw();
}

void CAddressDialog::OnEncoderCW(void)
{
	if (in_edit) {
		if (address >= configure_file.max_rs485_address) {
			beep(10);
			return;
		} else {
			++address;
		}
	} else {
		switch (menu_item) {
		default:
		case adr:
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

void CAddressDialog::OnEncoderCCW(void)
{
	if (in_edit) {
		if (address <= configure_file.min_rs485_address) {
			beep(10);
			return;
		} else {
			--address;
		}
	} else {
		switch (menu_item) {
		case adr:
			beep(10);
			return;
			break;
		case save:
			menu_item = adr;
			break;
		default:
		case cancel:
			menu_item = save;
			break;
		}
	}

	OnDraw();
}

void CAddressDialog::OnButtonDown(void)
{
	flash_file * p_flash_file;

	switch (menu_item) {
	default:
		menu_item = adr;
	case adr:
		in_edit = !in_edit;
		break;
	case save:
		fseek(p_flash, (int) (&p_flash_file->rs485_address) - (int) (p_flash_file),
		      SEEK_SET);
		fwrite(&address, sizeof(flash_file::rs485_address), 1, p_flash);
		fflush(p_flash);
	case cancel:
		p_dialog = &SettingDialog;
		p_dialog->OnCreat();
		return;
		break;
	}

	OnDraw();
}

void CAddressDialog::OnDraw(void)
{
	fseek(p_lcd, 102, SEEK_SET);
	fwrite(zeros, 3, 1, p_lcd);

	fseek(p_lcd, 112, SEEK_SET);
	fwrite(zeros, 16, 1, p_lcd);

	fseek(p_lcd, 38, SEEK_SET);
	fprintf(p_lcd, "%3i", address);

	switch (menu_item) {
	default:
		menu_item = adr;
	case adr:
		fseek(p_lcd, 102, SEEK_SET);
		if (in_edit) {
			fputs("\2\2\2", p_lcd);
		} else {
			fputs("\1\1\1", p_lcd);
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

