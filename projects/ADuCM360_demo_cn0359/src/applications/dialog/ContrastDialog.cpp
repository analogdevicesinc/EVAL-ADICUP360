/***************************************************************************//**
 *   @file   ContrastDialog.cpp
 *   @brief  Implementation of ContrastDialog.cpp
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

#include <applications/dialog/ContrastDialog.h>
#include <hal/devices.h>
#include <cstdio>
#include <hal/drivers/buzzer.h>
#include <applications/dialog/SettingDialog.h>

CContrastDialog ContrastDialog;

#define CONTRAST_STEP 5

void CContrastDialog::OnCreat(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);

	flash_file * p_flash_file;
	fseek(p_flash, (int) (&p_flash_file->brightness) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&brightness, sizeof(flash_file::brightness), 1, p_flash);

	OnDraw();
}

void CContrastDialog::OnEncoderCW(void)
{
	brightness += CONTRAST_STEP;

	if (brightness > configure_file.max_brightness) {
		brightness = configure_file.max_brightness;
		beep(10);
	}

	flash_file * p_flash_file;

	fseek(p_flash, (int) (&p_flash_file->brightness) - (int) (p_flash_file),
	      SEEK_SET);
	fwrite(&brightness, sizeof(flash_file::brightness), 1, p_flash);

	fflush(p_flash);
	FILE * p_file;
	p_file = freopen("lcd", "rb+", p_lcd);
	setvbuf(p_lcd, nullptr, _IOFBF, 16);

	OnDraw();
}

void CContrastDialog::OnEncoderCCW(void)
{
	brightness -= CONTRAST_STEP;

	if (brightness < configure_file.min_brightness) {
		brightness = configure_file.min_brightness;
		beep(10);
	}

	flash_file * p_flash_file;

	fseek(p_flash, (int) (&p_flash_file->brightness) - (int) (p_flash_file),
	      SEEK_SET);
	fwrite(&brightness, sizeof(flash_file::brightness), 1, p_flash);

	fflush(p_flash);

	FILE * p_file;
	p_file = freopen("lcd", "rb+", p_lcd);
	setvbuf(p_lcd, nullptr, _IOFBF, 16);

	OnDraw();
}

void CContrastDialog::OnButtonDown(void)
{
	p_dialog = &SettingDialog;
	p_dialog->OnCreat();
}

void CContrastDialog::OnDraw(void)
{
	rewind(p_lcd);
	fputs("LCD contrast:", p_lcd);

	fseek(p_lcd, 16, SEEK_SET);
	fputs("turn knob to", p_lcd);

	fseek(p_lcd, 32, SEEK_SET);
	fputs("adjust contrast", p_lcd);

	fseek(p_lcd, 53, SEEK_SET);
	fputs("Return", p_lcd);

	fseek(p_lcd, 117, SEEK_SET);
	fputs("\x1\x1\x1\x1\x1\x1", p_lcd);

	fflush(p_lcd);
}

