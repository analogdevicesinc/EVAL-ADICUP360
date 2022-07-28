/***************************************************************************//**
 *   @file   HomeDialog.cpp
 *   @brief  Implementation of HomeDialog.cpp
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

#include <applications/dialog/HomeDialog.h>
#include <applications/dialog/SettingDialog.h>
#include <hal/devices.h>
#include <cstdio>
#include <hal/drivers/adc.h>
#include <cmath>
#include <hal/drivers/buzzer.h>

CHomeDialog HomeDialog;

using namespace std;

void CHomeDialog::OnCreat(void)
{
	conductivity = 0;
	temperature = 25;
	rtd_wire = -1;
	rtd_type = -1;

	OnDraw();
}

void CHomeDialog::OnButtonDown(void)
{
	p_dialog = &SettingDialog;
	p_dialog->OnCreat();
}

void CHomeDialog::OnConductivity(void)
{
	adc_file adc;
	decltype(flash_file::cell_const) k;
	decltype(flash_file::temp_coef) conf;

	fseek(p_flash, (int) (&p_flash_file->cell_const) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&k, sizeof(flash_file::cell_const), 1, p_flash);

	fseek(p_flash, (int) (&p_flash_file->temp_coef) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&conf, sizeof(flash_file::temp_coef), 1, p_flash);

	rewind(p_adc);
	fread(&adc, sizeof(adc_file), 1, p_adc);

	conductivity = (k * (adc.p_curt - adc.n_curt) / (adc.p_volt - adc.n_volt))
		       * (100 / (100 + conf * (temperature - 25)));

	if (!(isfinite(conductivity) && conductivity >= 1e-24 && conductivity < 1e27)) {
		beep(100);
	}

	OnDraw();
}

void CHomeDialog::OnRTD(void)
{
	adc_file adc;
	decltype(flash_file::cell_const) k;
	decltype(flash_file::temp_coef) conf;

	fseek(p_flash, (int) (&p_flash_file->cell_const) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&k, sizeof(flash_file::cell_const), 1, p_flash);

	fseek(p_flash, (int) (&p_flash_file->temp_coef) - (int) (p_flash_file),
	      SEEK_SET);
	fread(&conf, sizeof(flash_file::temp_coef), 1, p_flash);

	rewind(p_adc);
	fread(&adc, sizeof(adc_file), 1, p_adc);

	temperature = adc.temp;
	rtd_wire = adc.wire_mode;
	rtd_type = adc.rtd_type;

	conductivity = (k * (adc.p_curt - adc.n_curt) / (adc.p_volt - adc.n_volt))
		       * (100 / (100 + conf * (temperature - 25)));

	if (rtd_wire == -1) {
		beep(10);
	}

	OnDraw();
}

void CHomeDialog::OnDraw(void)
{
	rewind(p_lcd);
	fwrite(zeros, 128, 1, p_lcd);
	rewind(p_lcd);

	if (rtd_wire == -1) {
		fputs("   RTD Error    ", p_lcd);
		fputs("    use 25\xf8" "C", p_lcd);
	} else {
		fprintf(p_lcd, " RTD: %iW PT%i", rtd_wire, rtd_type);
		fseek(p_lcd, 20, SEEK_SET);
		fprintf(p_lcd, "%6.2f\xf8" "C", temperature);
	}

	fseek(p_lcd, 32, SEEK_SET);
	fputs("Conductivity:", p_lcd);

	fseek(p_lcd, 51, SEEK_SET);

	if (isfinite(conductivity) && conductivity >= 1e-24 && conductivity < 1e27) {
		char prefix[] = "y\0z\0\a\0\f\0p\0n\0u\0m\0\0\0k\0M\0G\0T\0P\0E\0Z\0Y";
		int index = floor(log10(conductivity) / 3);

		float f = conductivity / pow(1000, index);

		if (f < 10) {
			fprintf(p_lcd, "%6.4f%sS/cm", f, &prefix[index * 2 + 16]);
		} else {
			if (f < 100) {
				fprintf(p_lcd, "%6.3f%sS/cm", f, &prefix[index * 2 + 16]);
			} else {
				fprintf(p_lcd, "%6.2f%sS/cm", f, &prefix[index * 2 + 16]);
			}
		}
	} else {
		fputs("sensor error", p_lcd);
	}

	fflush(p_lcd);
}
