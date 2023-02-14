/***************************************************************************//**
 *   @file   SettingDialog.cpp
 *   @brief  Implementation of SettingDialog.cpp
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
#include <applications/dialog/HomeDialog.h>
#include <applications/dialog/SettingDialog.h>
#include <cstdio>
#include <hal/devices.h>
#include <hal/drivers/buzzer.h>
#include <applications/dialog/VoltageDialog.h>
#include <applications/dialog/FrequencyDialog.h>
#include <applications/dialog/CellConstantDialog.h>
#include <applications/dialog/CoefficientDialog.h>
#include <applications/dialog/BaudRateDialog.h>
#include <applications/dialog/AddressDialog.h>
#include <applications/dialog/ContrastDialog.h>
#include <applications/dialog/HoldTimeDialog.h>
#include <applications/dialog/SetupTimeDialog.h>

CSettingDialog SettingDialog;

struct {
	const char caption[18];
	CDialog * const p_dialog;
} static const menu[] = {{"EXC Voltage", &VoltageDialog},
	{"EXC Frequency", &FrequencyDialog},
	{"T-H setup time", &SetupTimeDialog},
	{"T-H hold time", &HoldTimeDialog},
	{"TEMP Coefficient", &CoefficientDialog},
	{"Cell Constant", &CellConstantDialog},
	{"RS-485 Baud Rate", &BaudRateDialog},
	{"RS-485 Address", &AddressDialog},
	{"Return To Home", &HomeDialog},
	{"LCD Contrast", &ContrastDialog},
	{"Firmware Info", &FirmwareDialog}
};

#define SIZE(a) (sizeof(a)/sizeof(a[0]))

CSettingDialog::CSettingDialog(void)
{
	first_item = 5; //Cell Constant
	selected_line = 3; //Return To Home
}

void CSettingDialog::OnCreat(void)
{
	OnDraw();
}

void CSettingDialog::OnEncoderCW(void)
{
	if (selected_line < 3) {
		++selected_line;
	} else {

		if (first_item < SIZE(menu) - 4) {
			++first_item;
		} else {
			beep(1);
		}
	}

	OnDraw();
}

void CSettingDialog::OnEncoderCCW(void)
{
	if (selected_line > 0) {
		--selected_line;
	} else {
		if (first_item > 0) {
			--first_item;
		} else {
			beep(1);
		}
	}

	OnDraw();
}

void CSettingDialog::OnButtonDown(void)
{
	p_dialog = menu[first_item + selected_line].p_dialog;
	p_dialog->OnCreat();
}

void CSettingDialog::OnDraw(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);
	rewind(p_lcd);

	for (int y = 0; y < 4; ++y) {
		fseek(p_lcd, y * 16, SEEK_SET);
		fprintf(p_lcd, "%s", menu[first_item + y].caption);
	}

	fseek(p_lcd, selected_line * 16 + 64, SEEK_SET);

	fputs("\1\1\1\1\1\1\1\1\1\1\1\1\1\1\1\1", p_lcd);

	fflush(p_lcd);
}
