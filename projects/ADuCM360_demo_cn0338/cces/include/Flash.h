/**
******************************************************************************
*   @file     Flash.h
*   @brief    Header file for flash memory management.
*   @version  V0.1
*   @author   ADI
*   @date     February 2016
*  @par Revision History:
*  - V0.1, November 2016: initial version.
*
*******************************************************************************
* Copyright 2016(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
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
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT, MERCHANTABILITY
* AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************
*/

#pragma once

#include <climits>

struct SOptions {
   volatile unsigned int baud_rate;

   volatile float ndir_frequency;

   volatile float adc_frequency;

   volatile unsigned int rising_ms;

   volatile unsigned int falling_ms;

   volatile float zero; // FA = 1 - (I/zero) = span * (1 - Exp[-b * (x^c)]), x is concentration

   volatile float span;

   volatile float b;

   volatile float c;

   volatile float k; // kelvin temperature at calibrating

   volatile unsigned long serial_num;

   volatile bool init;

   SOptions(void);
};

extern SOptions Flash_GetOptions(void);

extern void Flash_SetOptions(const SOptions options);

extern bool Flash_VerifyOptions(void);

extern void Flash_Init(void);


#define MAX_NDIR_FREQ 5.f
#define MIN_NDIR_FREQ 0.1f

#define MAX_ADC_FREQ 483.f
#define MIN_ADC_FREQ 3.5f

#define MAX_SERIAL_NUM 0xfffffff0
#define MIN_SERIAL_NUM 0x10

#define MAX_RISING_EDGE 3000u

#define MAX_FALLING_EDGE 3000u

#define MAX_B_VALUE 10.f
#define MIN_B_VALUE 1.e-5f

#define MAX_C_VALUE 10.f
#define MIN_C_VALUE 0.1f


