/***************************************************************************//**
 *   @file   FirmwareDialog.cpp
 *   @brief  Implementation of FirmwareDialog.cpp
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
#include <applications/dialog/FirmwareDialog.h>
#include <applications/dialog/SettingDialog.h>
#include <cstdio>
#include <hal/devices.h>
#include <hal/drivers/buzzer.h>

CFirmwareDialog FirmwareDialog;

void CFirmwareDialog::OnCreat(void)
{
	first_line = 0;

	OnDraw();
}

void CFirmwareDialog::OnEncoderCW(void)
{
	if (first_line < ((sizeof(configure_file.version) + 14) / 16) - 4) {
		++first_line;
	} else {
		beep(10);
	}

	OnDraw();
}

void CFirmwareDialog::OnEncoderCCW(void)
{
	if (first_line > 0) {
		--first_line;
	} else {
		beep(10);
	}

	OnDraw();
}

void CFirmwareDialog::OnButtonDown(void)
{
	p_dialog = &SettingDialog;
	p_dialog->OnCreat();
}

void CFirmwareDialog::OnDraw(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);

	rewind(p_lcd);

	fprintf(p_lcd, "%.64s", &configure_file.version[first_line * 16]);

	fflush(p_lcd);
}
